#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import json


def multiply(poses_file, out_file):
    # with open(poses_file, 'r') as f:
    #     data = f.read().splitlines()
    #     print(data)
    data = np.genfromtxt(poses_file, dtype=float)
    d = data * 20
    np.savetxt(out_file, d)


if __name__ == "__main__":
   pf = '/home/zuyuan/rasberry_ws/src/data_kitti/OUTPUT/Traj_Vis/seq02_5-6_13-14/agentB_Est.txt'
   out_f = '/home/zuyuan/rasberry_ws/src/data_kitti/OUTPUT/Traj_Vis/seq02_5-6_13-14/agentB_Est_x20.txt'
   multiply(pf, out_f)


