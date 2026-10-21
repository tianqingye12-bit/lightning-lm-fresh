"""Cross-language contract test, invoked by run_scan_trajectory.py."""
from pathlib import Path
import sys
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from pose_trajectory import load_trajectory
from clean_map_vehicles import Trajectory
from clean_map_runtime import load_frame_contract

directory=Path(sys.argv[1])
cfg=dict(max_kf_gap=3,pose_trajectory=dict(source='dense',max_gap_s=.2))
traj=load_trajectory(directory,cfg,Trajectory)
assert traj.source=='dense' and traj.description['rows']==23 and traj.description['usable_rows']==22
expected=np.array([5+np.cos(.4)-np.sin(.4),np.sin(.4)+np.cos(.4),0])
np.testing.assert_allclose(traj.pose(11)[:3,3],expected,atol=1e-10)
assert np.linalg.norm(traj.pose(11)[:3,3]-traj.keyframes.pose(11)[:3,3])>.5
assert traj.pose(11.25) is None and traj.pose(11.3) is None and traj.pose(11.35) is None
assert traj.pose(9.8) is None and traj.pose(12.2) is None
assert traj.pose(10.55) is not None
for t in (10,12):np.testing.assert_allclose(traj.pose(t),traj.keyframes.pose(t),atol=1e-10)
T,_=load_frame_contract(directory)
camera_lidar=traj.pose(11)@T
np.testing.assert_allclose(camera_lidar[:3,3],expected+traj.pose(11)[:3,:3]@T[:3,3])
print('C++ writer -> Python reader: optimized anchors, curved scan motion, rejected-scan barriers and IMU/LiDAR contract passed')
