"""Map-consistent scan trajectory with explicit invalid-scan interpolation barriers."""
from array import array
import csv
from pathlib import Path

import numpy as np
import yaml
from clean_map_runtime import file_digest, load_frame_contract

DENSE_FILES = ('trajectory.yaml', 'scan_trajectory.csv', 'trajectory_anchors.csv')
POSE_FIELDS = ('tx', 'ty', 'tz', 'qx', 'qy', 'qz', 'qw')
MODES = ('anchor', 'head_hold', 'tail_hold', 'interpolated_correction')


def trajectory_options(cfg):
    options = dict(source='auto', max_gap_s=.2)
    supplied = cfg.get('pose_trajectory', {})
    if not isinstance(supplied, dict) or set(supplied)-set(options):
        raise ValueError('Unknown pose_trajectory option')
    options.update(supplied)
    if options['source'] not in ('auto', 'dense', 'keyframes'):
        raise ValueError('pose_trajectory.source must be auto, dense or keyframes')
    gap = options['max_gap_s']
    if isinstance(gap, bool) or not isinstance(gap, (int, float)) or not np.isfinite(gap) or gap <= 0 or gap > 2:
        raise ValueError('pose_trajectory.max_gap_s must be numeric in (0, 2]')
    return options


def pose_matrix(position, q):
    x, y, z, w = q
    pose = np.eye(4)
    pose[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                   [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                   [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    pose[:3, 3] = position
    return pose


def interpolate(p, q, i, j, alpha):
    q0, q1 = q[i], q[j]
    dot = np.dot(q0, q1)
    if dot < 0: q1, dot = -q1, -dot
    if dot > .9995:
        rotation = q0 + alpha*(q1-q0)
    else:
        angle = np.arccos(np.clip(dot, -1, 1))
        rotation = (np.sin((1-alpha)*angle)*q0 + np.sin(alpha*angle)*q1)/np.sin(angle)
    return pose_matrix((1-alpha)*p[i]+alpha*p[j], rotation/np.linalg.norm(rotation))


class PoseTrajectory:
    """Keep keyframe t/p/q/ids for manifest and floor; query dense poses separately."""
    def __init__(self, directory, cfg, keyframes):
        self.keyframes = keyframes
        self.t, self.p, self.q, self.ids = keyframes.t, keyframes.p, keyframes.q, keyframes.ids
        self.options = trajectory_options(cfg)
        self.source = 'keyframes'
        self.query_stats = dict(queries=0, exact=0, interpolated=0, outside_range=0,
                                invalid_scan=0, pose_gap=0, nonfinite_time=0)
        self.failure_samples = []
        self._failure_sample_counts = {}
        self.description = dict(source='keyframes', max_gap_s=float(keyframes.max_gap))
        directory = Path(directory)
        present = [(directory/name).exists() for name in DENSE_FILES]
        if self.options['source'] != 'keyframes' and any(present):
            if not all(present):
                raise ValueError('Partial dense trajectory export; restore the complete map or explicitly select keyframes')
            self._load_dense(directory)
        elif self.options['source'] == 'dense':
            raise ValueError('Dense trajectory missing. Rebuild with trajectory_recording.enabled and save a NEW map')
        self.pose_t = self.dense_t if self.source == 'dense' else self.t
        self.pose_p = self.dense_p if self.source == 'dense' else self.p
        self.pose_q = self.dense_q if self.source == 'dense' else self.q
        self.max_gap = self.options['max_gap_s'] if self.source == 'dense' else keyframes.max_gap

    def _load_dense(self, directory):
        _, contract = load_frame_contract(directory)
        if contract != 'imu_v2':
            raise ValueError('Dense IMU trajectory requires the version-2 map frame contract')
        metadata = yaml.safe_load((directory/'trajectory.yaml').read_text())
        required = dict(format_version=1, pose_frame='imu', coordinate_frame='exported_map',
                        timestamp_reference='lidar_end', correction_method='interpolated_keyframe_correction')
        if not isinstance(metadata, dict) or any(metadata.get(k) != v for k,v in required.items()):
            raise ValueError('Unsupported dense trajectory coordinate/version contract')
        if metadata.get('complete') is not True or metadata.get('recording_faults') != 0:
            raise ValueError('Dense recording incomplete (capacity/time fault); no automatic sparse fallback')
        for key in ('rows', 'usable_rows', 'keyframes'):
            if isinstance(metadata.get(key), bool) or not isinstance(metadata.get(key), int) or metadata[key] < 0:
                raise ValueError(f'Invalid trajectory metadata count: {key}')
        if metadata['rows'] == 0 or metadata['rows'] > 2000000 or metadata['keyframes'] != len(self.t):
            raise ValueError('Dense trajectory is empty, excessive or belongs to a different map')
        with (directory/'trajectory_anchors.csv').open(newline='') as stream:
            reader = csv.DictReader(stream)
            anchors = list(reader)
        if len(anchors) != len(self.t):
            raise ValueError('Dense anchor count differs from keyframes.txt')
        for i,row in enumerate(anchors):
            pose = np.array([float(row[k]) for k in POSE_FIELDS])
            raw = np.array([float(row['raw_'+k]) for k in POSE_FIELDS])
            if (int(row['id']) != self.ids[i] or abs(float(row['timestamp'])-self.t[i]) > 1e-6 or
                    not np.isfinite(pose).all() or not np.isfinite(raw).all() or
                    abs(np.linalg.norm(pose[3:])-1) > 1e-5 or abs(np.linalg.norm(raw[3:])-1) > 1e-5 or
                    np.linalg.norm(pose[:3]-self.p[i]) > 1e-5 or
                    abs(abs(np.dot(pose[3:],self.q[i]))-1) > 1e-6):
                raise ValueError('Dense correction anchors do not match the saved map poses')
        values, usable, modes = array('d'), array('B'), array('B')
        status_counts = {}
        timing = dict(imu_gap_scans=0, nonfinite_point_offsets=0, negative_point_offsets=0,
                      point_end_mismatch_scans=0, clock_mismatch_scans=0)
        with (directory/'scan_trajectory.csv').open(newline='') as stream:
            for i,row in enumerate(csv.DictReader(stream)):
                if i >= metadata['rows'] or int(row['scan_id']) != i or row['usable'] not in ('0','1'):
                    raise ValueError('Invalid dense trajectory row count, ID or usable flag')
                vector = [float(row[k]) for k in ('timestamp',*POSE_FIELDS)]
                if not np.isfinite(vector).all() or abs(np.linalg.norm(vector[4:])-1) > 1e-5:
                    raise ValueError('Invalid dense pose values')
                valid = int(row['usable'])
                clock_error = float(row['clock_error_s'])
                counters = [int(row['point_nonfinite']),int(row['point_negative'])]
                if min(counters)<0 or row['imu_gap'] not in ('0','1'):
                    raise ValueError('Invalid dense scan timing flags')
                bad_points = sum(counters)
                if valid and (row['status'] not in ('accepted','bootstrap') or row['imu_gap'] != '0' or
                              bad_points or not np.isfinite(clock_error) or abs(clock_error) > 1e-5):
                    raise ValueError('Dense usable flag contradicts scan quality')
                values.extend(vector); usable.append(valid); modes.append(MODES.index(row['correction_mode']))
                status_counts[row['status']] = status_counts.get(row['status'],0)+1
                timing['imu_gap_scans'] += int(row['imu_gap'])
                timing['nonfinite_point_offsets'] += int(row['point_nonfinite'])
                timing['negative_point_offsets'] += int(row['point_negative'])
                timing['point_end_mismatch_scans'] += int(abs(float(row['point_end_difference_s'])) > .002)
                timing['clock_mismatch_scans'] += int(not np.isfinite(clock_error) or abs(clock_error)>1e-5)
        data = np.frombuffer(values, dtype=np.float64).reshape(-1,8)
        self.dense_t, self.dense_p, self.dense_q = data[:,0], data[:,1:4], data[:,4:8]
        self.usable, self.correction_modes = np.frombuffer(usable,dtype=np.uint8), np.frombuffer(modes,dtype=np.uint8)
        if (len(data) != metadata['rows'] or int(self.usable.sum()) != metadata['usable_rows'] or
                np.any(np.diff(self.dense_t) <= 0)):
            raise ValueError('Dense trajectory truncated, non-increasing or inconsistent with metadata')
        # Every graph anchor must be represented exactly in this recording.
        for i,t in enumerate(self.t):
            j = int(np.searchsorted(self.dense_t,t))
            if (j == len(data) or abs(self.dense_t[j]-t)>1e-6 or not self.usable[j] or
                    np.linalg.norm(self.dense_p[j]-self.p[i])>1e-5 or
                    abs(abs(np.dot(self.dense_q[j],self.q[i]))-1)>1e-6):
                raise ValueError('Dense trajectory does not reproduce a saved keyframe pose')
        self.source = 'dense'
        self.description = dict(source='dense', max_gap_s=float(self.options['max_gap_s']),
            rows=len(data), usable_rows=int(self.usable.sum()), statuses=status_counts, timing=timing,
            correction_method=metadata['correction_method'], hashes={name:file_digest(directory/name) for name in DENSE_FILES})

    def _lookup(self, timestamp):
        if not np.isfinite(timestamp): return 0,0,0.,'nonfinite_time'
        j = int(np.searchsorted(self.pose_t,timestamp))
        if j < len(self.pose_t) and self.pose_t[j] == timestamp:
            if self.source == 'dense' and not self.usable[j]: return j,j,0.,'invalid_scan'
            return j,j,0.,'exact'
        if j == 0 or j == len(self.pose_t): return max(0,j-1),min(j,len(self.pose_t)-1),0.,'outside_range'
        if self.source == 'dense' and (not self.usable[j-1] or not self.usable[j]): return j-1,j,0.,'invalid_scan'
        gap = self.pose_t[j]-self.pose_t[j-1]
        if gap > self.max_gap+1e-9: return j-1,j,0.,'pose_gap'
        return j-1,j,float((timestamp-self.pose_t[j-1])/gap),'interpolated'

    def pose(self, timestamp):
        i,j,alpha,status = self._lookup(timestamp)
        self.query_stats['queries'] += 1; self.query_stats[status] += 1
        if status not in ('exact','interpolated'):
            if len(self.failure_samples)<200 and self._failure_sample_counts.get(status,0)<40:
                self.failure_samples.append(dict(timestamp=float(timestamp) if np.isfinite(timestamp) else None,
                                                 **self.association_info(timestamp)))
                self._failure_sample_counts[status]=self._failure_sample_counts.get(status,0)+1
            return None
        return interpolate(self.pose_p,self.pose_q,i,j,alpha)

    def association_info(self, timestamp):
        i,j,_,status = self._lookup(timestamp)
        nearest = float(min(abs(self.pose_t[i]-timestamp),abs(self.pose_t[j]-timestamp))) if np.isfinite(timestamp) else None
        correction = ''
        if self.source == 'dense':
            correction = MODES[self.correction_modes[i]] if self.correction_modes[i] == self.correction_modes[j] else 'mixed_corrections'
        return dict(pose_source=self.source, pose_status=status, nearest_pose_dt_s=nearest,
                    pose_bracket_gap_s=float(self.pose_t[j]-self.pose_t[i]), pose_correction=correction)


def load_trajectory(directory, cfg, keyframe_type):
    keyframes = keyframe_type(Path(directory)/'keyframes.txt',cfg['max_kf_gap'])
    return PoseTrajectory(directory,cfg,keyframes)
