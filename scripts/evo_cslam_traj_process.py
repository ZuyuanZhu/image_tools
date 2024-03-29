#!/usr/bin/env python

from evo_traj_time_sort import TrajectorySorter
from evo_traj_time_sort_merger import MoveTrajectoryTimestamp

"""
This is a pre-process before evaluating the estimated trajectories generated by CCMSLAM.
Firstly, reverse the trajectory timestamp of the two agents, so the timestamp is ascending.
Secondly, change the trajectory timestamp of agentB so B's timestamp is continuous just after
    A's last timestamp.
"""

if __name__ == "__main__":
    # change the values
    base_path = "/home/zuyuan/Documents/dataset/kitti/Traj_Vis/Self_trained_ORBvoc_seq00/seq00_1-2_10-11/"
    seq = '00'

    # reverse the trajectory timestamp of the two agents, so the timestamp is ascending
    for agent_index in ['0', '1']:
        est_file = "KF_GBA_{}.csv".format(agent_index)
        est_file_sorted = "KF_GBA_{}_sorted.csv".format(agent_index)
        sorter = TrajectorySorter(base_path, est_file, est_file_sorted)
        sorter.sort_trajectory()

    # Secondly, change the trajectory timestamp of agentB so B's timestamp is continuous just after
    #     A's last timestamp.
    agentA_traj_sort = "KF_GBA_0_sorted.csv"
    agentB_traj_sort = "KF_GBA_1_sorted.csv"
    agentB_traj_cont = 'KF_GBA_1_sorted_cont.csv'
    combined_traj_sort = 'KF_GBA_0_1_sorted_cont.csv'
    groundTruth_orig = "{}_time_poses_selected.csv".format(seq)
    groundTruth_cont = '{}_time_poses_selected_con.csv'.format(seq)
    mover = MoveTrajectoryTimestamp(base_path,
                                    agentA_traj_sort, agentB_traj_sort,
                                    groundTruth_orig, groundTruth_cont,
                                    agentB_traj_cont)
    mover.move_traj_timestamp()
    # mover.merge_trajectories(combined_traj_sort)
