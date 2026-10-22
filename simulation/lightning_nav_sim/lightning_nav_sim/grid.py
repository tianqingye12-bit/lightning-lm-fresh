"""Nav2 trinary PGM geometry; arrays use ROS bottom-left row order."""
from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
import re

import numpy as np
import yaml

MAX_CELLS = 8_000_000


def read_pgm(path):
    # Parse only the header: a binary pixel may itself be whitespace or '#'.
    with Path(path).open('rb') as stream:
        def token():
            word = bytearray()
            while True:
                c = stream.read(1)
                if not c:
                    raise ValueError('Truncated PGM')
                if c == b'#' and not word:
                    stream.readline()
                elif c.isspace():
                    if word:
                        if c == b'\r' and stream.peek(1)[:1] == b'\n':
                            stream.read(1)
                        return bytes(word)
                else:
                    word.extend(c)
        magic, width, height, maximum = token(), int(token()), int(token()), int(token())
        if magic not in (b'P5', b'P2') or maximum != 255:
            raise ValueError('Use an 8-bit P5/P2 PGM map (maxval=255)')
        if min(width, height) <= 0 or width * height > MAX_CELLS:
            raise ValueError('PGM dimensions exceed the 8 million cell budget')
        if magic == b'P5':
            body = stream.read(width * height + 1)
            if len(body) != width * height:
                raise ValueError('PGM body length differs from dimensions')
            pixels = np.frombuffer(body, dtype=np.uint8)
        else:
            body = stream.read(MAX_CELLS * 4 + 1)
            values = re.sub(rb'#[^\r\n]*', b'', body).split()
            if len(values) != width * height:
                raise ValueError('PGM body length differs from dimensions')
            pixels = np.array([int(v) for v in values], dtype=np.int64)
            if np.any((pixels < 0) | (pixels > 255)):
                raise ValueError('Invalid PGM pixel')
        return pixels.astype(np.uint8).reshape(height, width)


@dataclass
class Grid:
    cells: np.ndarray
    resolution: float
    origin: tuple

    def __post_init__(self):
        self.cells = np.asarray(self.cells, dtype=np.int8)
        self.origin = tuple(float(v) for v in self.origin)
        if (self.cells.ndim != 2 or not self.cells.size or self.cells.size > MAX_CELLS
                or len(self.origin) != 3 or not np.isfinite(self.origin).all()
                or not math.isfinite(self.resolution) or self.resolution <= 0
                or not np.isin(self.cells, [-1, 0, 100]).all()):
            raise ValueError('Invalid trinary grid geometry')

    def local(self, xy):
        xy = np.asarray(xy, dtype=float) - self.origin[:2]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return xy @ np.array([[c, -s], [s, c]])

    def world(self, xy):
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return np.asarray(xy) @ np.array([[c, s], [-s, c]]) + self.origin[:2]

    def blocked(self, xy):
        indices = np.floor(self.local(xy) / self.resolution).astype(np.int64)
        x, y = indices[..., 0], indices[..., 1]
        inside = (x >= 0) & (y >= 0) & (x < self.cells.shape[1]) & (y < self.cells.shape[0])
        return ~inside | (self.cells[np.clip(y, 0, self.cells.shape[0]-1),
                                     np.clip(x, 0, self.cells.shape[1]-1)] != 0)

    def collides(self, pose, radius):
        x, y = self.local(np.asarray(pose)[:2])
        h, w = self.cells.shape
        r = self.resolution
        if x-radius <= 0 or y-radius <= 0 or x+radius >= w*r or y+radius >= h*r:
            return True
        left, right = int((x-radius)//r), int((x+radius)//r)
        bottom, top = int((y-radius)//r), int((y+radius)//r)
        ys, xs = np.nonzero(self.cells[bottom:top+1, left:right+1] != 0)
        dx = np.maximum(np.maximum((xs+left)*r-x, x-(xs+left+1)*r), 0)
        dy = np.maximum(np.maximum((ys+bottom)*r-y, y-(ys+bottom+1)*r), 0)
        return bool(np.any(dx*dx+dy*dy <= radius*radius))

    def fingerprint(self):
        meta = np.asarray([self.resolution, *self.origin, *self.cells.shape], dtype='<f8').tobytes()
        return hashlib.sha256(meta+self.cells.tobytes()).hexdigest()


def load_grid(path):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    if cfg.get('mode', 'trinary') != 'trinary' or cfg.get('negate', 0) not in (0, 1):
        raise ValueError('Only Nav2 trinary maps are supported')
    free, occupied = float(cfg['free_thresh']), float(cfg['occupied_thresh'])
    if not 0 <= free < occupied <= 1:
        raise ValueError('Invalid occupancy thresholds')
    pixels = read_pgm(path.parent / cfg['image'])[::-1].astype(float)
    probability = pixels/255. if cfg.get('negate', 0) else 1.-pixels/255.
    cells = np.full(pixels.shape, -1, dtype=np.int8)
    cells[probability < free] = 0
    cells[probability > occupied] = 100
    return Grid(cells, float(cfg['resolution']), cfg['origin'])


def save_grid(grid, directory, name='map'):
    directory = Path(directory)
    pixels = np.where(grid.cells == 100, 0, np.where(grid.cells == 0, 254, 205)).astype(np.uint8)
    h, w = pixels.shape
    with (directory / (name+'.pgm')).open('xb') as stream:
        stream.write(f'P5\n{w} {h}\n255\n'.encode()+pixels[::-1].tobytes())
    config = dict(image=name+'.pgm', resolution=float(grid.resolution), origin=list(grid.origin),
                  negate=0, occupied_thresh=.65, free_thresh=.196, mode='trinary')
    with (directory / (name+'.yaml')).open('x') as stream:
        yaml.safe_dump(config, stream, sort_keys=False)
    return directory / (name+'.yaml')
