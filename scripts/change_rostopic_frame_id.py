import rospy
import rosbag
import numpy as np

"""
1. Assign sequence number to frame.id 
2. Split the rosbag at the specific number
"""

# Open the bag file for reading
bag_path = '/media/zuyuan/DATA1TB/Jackal/bags_data_campaign_july_2023/'
bag_name = 'test_2_long_134_258_select'
src_bag = bag_path + bag_name + ".bag"
bag = rosbag.Bag(src_bag, 'r')

# Create a new bag file for writing with modified frame IDs
new_bag = rosbag.Bag(bag_path + bag_name + '_frame_id_odom.bag', 'w', compression=rosbag.Compression.NONE)

# Initialize the frame ID counters
frame_id_counter = 40000
frame_id_counter_pose = 40000
split_num = 2887

# Flag to determine when to save the reset as a rosbag
save_reset = False

# Iterate over the messages in the original bag file
for topic, msg, t in bag.read_messages():
    # Check if the message is from the camera topic
    if topic == '/zed2/zed_node_test/left_raw/image_raw_gray':
        # Convert the frame ID counter to a zero-padded string
        new_frame_id = str(frame_id_counter).zfill(10)
        # Modify the frame ID of the camera message
        msg.header.frame_id = new_frame_id
        # Increment the frame ID counter
        frame_id_counter += 1
    elif topic == '/zed2/zed_node_test/odom':
        # Convert the frame ID counter to a zero-padded string
        new_frame_id_pose = str(frame_id_counter_pose).zfill(10)
        # Modify the frame ID of the position message
        msg.header.frame_id = new_frame_id_pose
        # Increment the frame ID counter
        frame_id_counter_pose += 1

    # Write the modified message to the new bag file
    new_bag.write(topic, msg, t)

    # # Check if the frame ID counter reaches the desired value
    # if frame_id_counter >= split_num and frame_id_counter_pose >= split_num and not save_reset:
    #     # Save the current bag as the first rosbag
    #     new_bag.close()
    #     save_reset = True
    #     new_bag = rosbag.Bag(bag_path + 'jackal_zed_small_Fri_1_0_agent_1_frame_id_odom.bag', 'w', compression=rosbag.Compression.NONE)

# Close the bag files
bag.close()
new_bag.close()
