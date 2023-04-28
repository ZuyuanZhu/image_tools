#!/usr/bin/env python


import numpy as np
from evo.tools.file_interface import read_tum_trajectory_file, write_tum_trajectory_file
from evo.core import trajectory
import csv
import io


base_path = "/home/zuyuan/Documents/dataset/kitti/Traj_Vis/Self_trained_ORBvoc_seq00/seq00_1-2_10-11/"
agentA_traj_sort = base_path + "KF_GBA_0_sorted.csv"
agentB_traj_sort = base_path + "KF_GBA_1_sorted.csv"
groundTruth_orig = base_path + "00_time_poses_selected.csv"

agentA_traj = read_tum_trajectory_file(agentA_traj_sort)
agentB_traj = read_tum_trajectory_file(agentB_traj_sort)
groundTruth = read_tum_trajectory_file(groundTruth_orig)

last_timestamp_A = agentA_traj.timestamps[-1]
first_timestamp_B = agentB_traj.timestamps[0]

agentB_traj.timestamps -= first_timestamp_B
agentB_traj.timestamps = agentB_traj.timestamps + last_timestamp_A + 0.3

gap_index = -1
gap_timeStamp = -1
for index, timeStamp in enumerate(groundTruth.timestamps):
    if -0.01 < timeStamp - first_timestamp_B < 0.01:
        gap_timeStamp = timeStamp
        gap_index = index
        break

if gap_index > 0:
    for idx in range(gap_index-1, len(groundTruth.timestamps)):
        groundTruth.timestamps[idx] = groundTruth.timestamps[idx] - gap_timeStamp + last_timestamp_A + 0.3

groundTruth_continuous = base_path + '00_time_poses_selected_con.csv'
write_tum_trajectory_file(groundTruth_continuous, groundTruth)
agentB_traj_con = base_path + 'KF_GBA_1_sorted_cont.csv'
write_tum_trajectory_file(agentB_traj_con, agentB_traj)

# Open both CSV files
with open(agentA_traj_sort, 'r') as a_file, open(agentB_traj_con, 'r') as b_file:
    # Read the contents of both files
    a_reader = csv.reader(a_file)
    b_reader = csv.reader(b_file)
    a_rows = list(a_reader)
    b_rows = list(b_reader)

# Combine the rows from both files
combined_rows = a_rows + b_rows

combined = base_path + 'KF_GBA_0_1_sorted_cont.csv'
# Write the combined rows to a new CSV file
with io.open(combined, 'wb') as combined_file:
    writer = csv.writer(combined_file)
    writer.writerows(combined_rows)

print("")

# sorted_indices = np.argsort(est_traj.timestamps)
# sorted_timestamps = est_traj.timestamps[sorted_indices]
# sorted_poses = [est_traj.poses_se3[i] for i in sorted_indices]
# sorted_traj = trajectory.PoseTrajectory3D(poses_se3=sorted_poses, timestamps=sorted_timestamps)
#
# write_tum_trajectory_file(sorted_tum_file, sorted_traj)

