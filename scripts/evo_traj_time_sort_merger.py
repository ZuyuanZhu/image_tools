#!/usr/bin/env python

import numpy as np
import csv
import io
from evo.tools.file_interface import read_tum_trajectory_file, write_tum_trajectory_file
from evo.core import trajectory


class MoveTrajectoryTimestamp:
    """
    Given two estimated trajectory generated by CCMSLAM. After sorting the timestamp, change the timestamp
    of agentB's trajectory just after agentA's last timestamp, so the combined trajectory of A and B are
    continuous. Here, the combined trajectory is not used for plotting. Only the timestamp-corrected trajectory
    of agentB is used in custom_app of evo pkg to plot the evaluation of the combined trajectory.
    """
    def __init__(self, base_path,
                 agentA_traj_sort, agentB_traj_sort,
                 groundTruth_orig, groundTruth_cont,
                 agentB_traj_cont):
        self.base_path = base_path
        self.agentA_traj_sort = self.base_path + agentA_traj_sort
        self.agentB_traj_sort = self.base_path + agentB_traj_sort
        self.agentB_traj_cont = self.base_path + agentB_traj_cont
        self.groundTruth_orig = self.base_path + groundTruth_orig
        self.groundTruth_continuous = self.base_path + groundTruth_cont

    def move_traj_timestamp(self):
        agentA_traj = read_tum_trajectory_file(self.agentA_traj_sort)
        agentB_traj = read_tum_trajectory_file(self.agentB_traj_sort)
        groundTruth = read_tum_trajectory_file(self.groundTruth_orig)

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

        write_tum_trajectory_file(self.groundTruth_continuous, groundTruth)
        write_tum_trajectory_file(self.agentB_traj_cont, agentB_traj)
        print("Trajectory timestamp changed successfully: {0}".format(self.agentB_traj_cont))

    def merge_trajectories(self, combined_traj_sort):
        with open(self.agentA_traj_sort, 'r') as a_file, open(self.agentB_traj_cont, 'r') as b_file:
            a_reader = csv.reader(a_file)
            b_reader = csv.reader(b_file)
            a_rows = list(a_reader)
            b_rows = list(b_reader)

        combined_rows = a_rows + b_rows
        combined = self.base_path + combined_traj_sort
        with io.open(combined, 'wb') as combined_file:
            writer = csv.writer(combined_file)
            writer.writerows(combined_rows)

        print("Trajectories merged successfully")


if __name__ == "__main__":
    base_path = "/home/zuyuan/Documents/dataset/kitti/Traj_Vis/Self_trained_ORBvoc_seq00/seq02_2-3.5_12-13/"
    agentA_traj_sort = "KF_GBA_0_sorted.csv"
    agentB_traj_sort = "KF_GBA_1_sorted.csv"
    agentB_traj_cont = 'KF_GBA_1_sorted_cont.csv'
    combined_traj_sort = 'KF_GBA_0_1_sorted_cont.csv'
    groundTruth_orig = "02_time_poses_selected.csv"
    groundTruth_cont = '00_time_poses_selected_con.csv'
    mover = MoveTrajectoryTimestamp(base_path,
                                    agentA_traj_sort, agentB_traj_sort,
                                    groundTruth_orig, groundTruth_cont,
                                    agentB_traj_cont)
    mover.move_traj_timestamp()
    mover.merge_trajectories(combined_traj_sort)
