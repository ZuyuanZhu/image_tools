import rospy
import rosbag
import numpy as np

# Open the bag file for reading
bag_path = '/media/zuyuan/DATA1TB/kitti/sim/'
bag_name = 'agent1_sim.bag'
src_bag = bag_path + bag_name
bag = rosbag.Bag(src_bag, 'r')

# Create a new bag file for writing with modified frame IDs
new_bag = rosbag.Bag(bag_path + 'agent1_sim_frame_id.bag', 'w', compression=rosbag.Compression.NONE)

# Initialize the frame ID counter
frame_id_counter = 0
frame_id_counter_pose = 0

# Initialize the position list
positions = []

# Iterate over the messages in the original bag file
for topic, msg, t in bag.read_messages():
    # Check if the message is from the camera topic
    if topic == '/cam0/image_raw':
        # Convert the frame ID counter to a zero-padded string
        new_frame_id = str(frame_id_counter).zfill(10)
        # Modify the frame ID of the camera message
        msg.header.frame_id = new_frame_id
        # Increment the frame ID counter
        frame_id_counter += 1
    if topic == '/leica/position':
        # Convert the frame ID counter to a zero-padded string
        new_frame_id_pose = str(frame_id_counter_pose).zfill(10)
        # Modify the frame ID of the position message
        msg.header.frame_id = new_frame_id_pose
        # Increment the frame ID counter
        frame_id_counter_pose += 1
        # Save the position data
        positions.append([msg.point.x, msg.point.y, msg.point.z])

    # Write the modified message to the new bag file
    new_bag.write(topic, msg, t)

# Convert the positions list to a NumPy array
positions = np.array(positions)

# Save the positions array to a local file
np.save(bag_path + 'agent1_sim_positions.npy', positions)

# Close the bag files
bag.close()
new_bag.close()
