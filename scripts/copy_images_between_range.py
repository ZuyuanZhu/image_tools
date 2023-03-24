#!/usr/bin/env python

import os
import shutil
import sys

from pathlib import Path


def copy_images_between_range(source_folder_, start_, end_, folder_name_):
    source_folder_ = source_folder_
    destination_folder = folder_name_

    # Create the destination folder if it doesn't exist
    if not Path(folder_name_).exists():
        Path(folder_name_).mkdir()
    # Path(destination_folder).mkdir(parents=False)

    for i in range(start_, end_ + 1):
        file_name = "%010d.png" % i
        src_path = os.path.join(source_folder_, file_name)
        dest_path = os.path.join(destination_folder, file_name)

        if os.path.exists(src_path):
            shutil.copy(src_path, dest_path)
        else:
            print("File %s not found in the source folder." % file_name)


if __name__ == "__main__":
    # It takes two integers and a custom folder name as input,
    # then copies the images with names between the two integers into the specified folder.
    # if len(sys.argv) != 5:
    #     print("Usage: python copy_images_between_range.py <source_folder> <start> <end> <folder_name>")
    #     sys.exit(1)
    #
    # source_folder = sys.argv[1]
    # start = int(sys.argv[2])
    # end = int(sys.argv[3])
    # folder_name = sys.argv[4]

    source_folder = '/media/zuyuan/DATA1TB/kitti/2011_10_03/2011_10_03_drive_0034_sync/image_00/data/'
    start = 3955
    end = 4288
    folder_name = '/media/zuyuan/DATA1TB/kitti/kitti_splited/2011_10_03/line12-13'

    copy_images_between_range(source_folder, start, end, folder_name)
    print("Images have been copied to the %s folder." % folder_name)
