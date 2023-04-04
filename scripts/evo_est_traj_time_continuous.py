#!/usr/bin/env python


import numpy as np
from evo.tools.file_interface import read_tum_trajectory_file, write_tum_trajectory_file
from evo.core import trajectory


base_path = "/home/zuyuan/rasberry_ws/src/data_kitti/OUTPUT/Traj_Vis/seq00_agentA0-320_agentB1378-1698/"
est_file = base_path + "KF_GBA_1.csv"
sorted_tum_file = base_path + "KF_GBA_1_sorted.csv"

est_traj = read_tum_trajectory_file(est_file)

sorted_indices = np.argsort(est_traj.timestamps)
sorted_timestamps = est_traj.timestamps[sorted_indices]
sorted_poses = [est_traj.poses_se3[i] for i in sorted_indices]
sorted_traj = trajectory.PoseTrajectory3D(poses_se3=sorted_poses, timestamps=sorted_timestamps)

write_tum_trajectory_file(sorted_tum_file, sorted_traj)

