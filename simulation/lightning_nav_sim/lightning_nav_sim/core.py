"""Fixed-step differential-drive model and independent-world laser/collision model.

Planar kinematics only: no suspension, 3D LiDAR, camera or IMU simulation.
"""
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .grid import load_grid


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def integrate(pose, v, w, dt):
    x, y, yaw = pose
    if abs(w) < 1e-9:
        return np.array([x+v*dt*math.cos(yaw), y+v*dt*math.sin(yaw), wrap(yaw+w*dt)])
    return np.array([x+v/w*(math.sin(yaw+w*dt)-math.sin(yaw)),
                     y-v/w*(math.cos(yaw+w*dt)-math.cos(yaw)), wrap(yaw+w*dt)])


def load_scenario(path):
    path = Path(path).resolve()
    cfg = yaml.safe_load(path.read_text())
    if not isinstance(cfg, dict) or not {'world', 'start', 'goals'} <= cfg.keys():
        raise ValueError('Scenario requires world, start and goals')
    for pose in [cfg['start'], *cfg['goals']]:
        if len(pose) != 3 or not np.isfinite(pose).all():
            raise ValueError('Poses must be finite [x,y,yaw] in the map coordinate frame')
    if not cfg['goals']:
        raise ValueError('At least one goal is required')
    cfg.setdefault('seed', 42)
    if isinstance(cfg['seed'], bool) or not isinstance(cfg['seed'], int) or cfg['seed'] < 0:
        raise ValueError('seed must be a nonnegative integer')
    cfg.setdefault('robot', {})
    defaults = dict(radius=.22, max_linear=.35, max_angular=1., dt=.05,
                    command_timeout=.5, odom_linear_scale=1.005, odom_angular_scale=1.01)
    cfg['robot'] = {**defaults, **cfg['robot']}
    if (not np.isfinite(list(cfg['robot'].values())).all()
            or any(v <= 0 for v in cfg['robot'].values())
            or not .01 <= cfg['robot']['dt'] <= .1):
        raise ValueError('Invalid robot parameters (dt must be 0.01..0.1s)')
    cfg['laser'] = {**dict(beams=180, range=8., noise_std=.01, period=.2), **cfg.get('laser', {})}
    laser = cfg['laser']
    if (not isinstance(laser['beams'], int) or not 16 <= laser['beams'] <= 1080
            or not np.isfinite(list(laser.values())).all() or not .5 <= laser['range'] <= 30.
            or not 0 <= laser['noise_std'] <= .2 or laser['period'] < cfg['robot']['dt']):
        raise ValueError('Invalid laser geometry/rate')
    for obstacle in cfg.setdefault('obstacles', []):
        for key in ('center', 'size'):
            if len(obstacle[key]) != 2 or not np.isfinite(obstacle[key]).all():
                raise ValueError('Obstacle center/size must be finite pairs')
        if min(obstacle['size']) <= 0:
            raise ValueError('Obstacle size must be positive')
        obstacle.setdefault('yaw', 0.)
        motion = obstacle.setdefault('motion', {})
        motion.update({**dict(axis='y', amplitude=0., period=20., phase=0.), **motion})
        if (motion['axis'] not in ('x','y') or not np.isfinite(
                [obstacle['yaw'], motion['amplitude'], motion['period'], motion['phase']]).all()
                or motion['amplitude'] < 0 or motion['period'] <= 0):
            raise ValueError('Invalid scripted obstacle motion')
    cfg['world'] = str((path.parent / cfg['world']).resolve())
    return cfg


class World:
    def __init__(self, scenario):
        self.cfg = scenario
        self.grid = load_grid(scenario['world'])
        if self.grid.resolution < .02:
            raise ValueError('Use world resolution >= 0.02m for the lightweight laser model')
        self.pose = np.array(scenario['start'], dtype=float)
        self.odom = np.zeros(3)
        self.elapsed = self.distance = 0.
        self.collisions = 0
        self.contact = False
        self.rng = np.random.default_rng(scenario['seed'])
        self.world_hash = self.grid.fingerprint()
        protocol = {k:v for k,v in scenario.items() if k != 'world'}
        self.protocol_hash = hashlib.sha256(json.dumps(protocol,sort_keys=True).encode()).hexdigest()
        if self.collides(self.pose, 0.):
            raise ValueError('Robot start footprint intersects occupied/unknown world cells or an obstacle')

    def boxes(self, timestamp):
        boxes = []
        for obstacle in self.cfg['obstacles']:
            center = np.array(obstacle['center'], dtype=float)
            m = obstacle['motion']
            center[('x','y').index(m['axis'])] += m['amplitude']*math.sin(2*math.pi*timestamp/m['period']+m['phase'])
            boxes.append((center, np.array(obstacle['size'])/2, obstacle['yaw']))
        return boxes

    def collides(self, pose, timestamp):
        radius = self.cfg['robot']['radius']
        if self.grid.collides(pose, radius):
            return True
        for center, half, yaw in self.boxes(timestamp):
            c, s = math.cos(yaw), math.sin(yaw)
            p = (np.asarray(pose[:2])-center) @ np.array([[c,-s],[s,c]])
            d = np.maximum(np.abs(p)-half, 0)
            if np.dot(d,d) <= radius*radius:
                return True
        return False

    def step(self, linear, angular):
        r = self.cfg['robot']
        if not np.isfinite([linear, angular]).all():
            linear = angular = 0.
        v, w = np.clip(linear, -r['max_linear'], r['max_linear']), np.clip(angular, -r['max_angular'], r['max_angular'])
        # Swept-circle substeps prevent jumping over walls at high commands.
        obstacle_speed=max((2*math.pi*o['motion']['amplitude']/o['motion']['period']
                            for o in self.cfg['obstacles']),default=0.)
        n = max(1, math.ceil((abs(v)+obstacle_speed)*r['dt']/min(self.grid.resolution/3, r['radius']/3)))
        if n > 1000:
            raise ValueError('Scenario motion exceeds the collision substep budget')
        dt = r['dt']/n
        contact = False
        moved = 0.
        for i in range(n):
            candidate = integrate(self.pose, v, w, dt)
            if self.collides(candidate, self.elapsed+(i+1)*dt):
                contact = True
                break
            self.pose = candidate
            self.odom = integrate(self.odom, v*r['odom_linear_scale'], w*r['odom_angular_scale'], dt)
            moved += abs(v)*dt
        self.distance += moved
        self.collisions += int(contact and not self.contact)
        self.contact = contact
        self.elapsed += r['dt']
        return (float(v), float(w)) if not contact else (0., 0.)

    def scan(self):
        cfg = self.cfg['laser']
        angles = np.linspace(-math.pi, math.pi, cfg['beams'], endpoint=False)
        direction = np.column_stack((np.cos(angles+self.pose[2]), np.sin(angles+self.pose[2])))
        # Half-cell ray marching. Unknown and world boundaries are solid.
        ranges = np.full(cfg['beams'], cfg['range'])
        active = np.ones(cfg['beams'], dtype=bool)
        for distance in np.arange(0., cfg['range']+self.grid.resolution/2, self.grid.resolution/2):
            hit = self.grid.blocked(self.pose[:2]+direction*distance) & active
            ranges[hit] = min(distance, cfg['range'])
            active[hit] = False
            if not active.any():
                break
        # Analytical ray/box slabs, including scripted moving vehicles.
        for center, half, yaw in self.boxes(self.elapsed):
            c, s = math.cos(yaw), math.sin(yaw)
            rotation = np.array([[c,-s],[s,c]])
            p, d = (self.pose[:2]-center) @ rotation, direction @ rotation
            parallel = np.abs(d) < 1e-12
            safe = np.where(parallel, 1., d)
            a, b = (-half-p)/safe, (half-p)/safe
            lower, upper = np.minimum(a,b), np.maximum(a,b)
            lower[parallel], upper[parallel] = -np.inf, np.inf
            near, far = lower.max(axis=1), upper.min(axis=1)
            valid = (far >= np.maximum(near, 0.)) & ~np.any(parallel & (np.abs(p)>half), axis=1)
            ranges = np.minimum(ranges, np.where(valid, np.maximum(near,0.), cfg['range']))
        hit = ranges < cfg['range']
        ranges[hit] += self.rng.normal(0.,cfg['noise_std'],int(hit.sum()))
        ranges = np.clip(ranges, .02, cfg['range'])
        ranges[~hit] = np.inf  # Clear free rays in Nav2; do not mark a max-range ring.
        return angles, ranges
