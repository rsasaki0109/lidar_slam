"""Initial pose of an update's odometry frame in the store frame (known start), from NCLT ground
truth: store_from_world (base session odometry fitted to its ground truth) times world_from_update
(the update session mapped alone, fitted to its ground truth)."""
import json, sys
import numpy as np
from scipy.spatial.transform import Rotation as R
sys.path.insert(0, '/home/user/lidar_slam_ros2/scripts')
from evaluate_luh_trajectory import fit
def world_from_run(run, day):
    est = np.loadtxt(f'runs/{run}/odometry_tum.txt'); gt = np.load(f'gt/{day}.npy'); t = gt[:, 0] * 1e-6
    i = np.searchsorted(t, est[:, 0]).clip(1, len(t) - 1); ok = np.abs(t[i] - est[:, 0]) < .5
    r, tr = fit(est[ok, 1:4], gt[i[ok], 1:4]); m = np.eye(4); m[:3, :3], m[:3, 3] = r, tr
    return m
if __name__ == '__main__':
    base_run, base_day, alone_run, update_day, out = sys.argv[1:6]
    store_from_world = np.linalg.inv(world_from_run(base_run, base_day))
    json.dump(store_from_world.tolist(), open(out, 'w'))
    pose = store_from_world @ world_from_run(alone_run, update_day)
    q = R.from_matrix(pose[:3, :3]).as_quat()
    print(' '.join(f'{v:.6f}' for v in [*pose[:3, 3], *q]))


def trimmed_prior(base_run, base_day, alone_run, update_day, start_sec):
    """Known start for a bag trimmed to begin at start_sec: the alone run's pose there."""
    store_from_world = np.linalg.inv(world_from_run(base_run, base_day))
    est = np.loadtxt(f'runs/{alone_run}/odometry_tum.txt')
    k = np.searchsorted(est[:, 0], start_sec)
    local = np.eye(4); local[:3, :3] = R.from_quat(est[k, 4:8]).as_matrix(); local[:3, 3] = est[k, 1:4]
    pose = store_from_world @ world_from_run(alone_run, update_day) @ local
    q = R.from_matrix(pose[:3, :3]).as_quat()
    return ' '.join(f'{v:.6f}' for v in [*pose[:3, 3], *q])
