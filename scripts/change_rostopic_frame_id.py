import rospy
import rosbag
from sensor_msgs.msg import Image

# Open the bag file for reading
bag_path = '/media/zuyuan/DATA1TB/kitti/sim/'
bag_name = 'agent1_sim.bag'
src_bag = bag_path + bag_name
bag = rosbag.Bag(src_bag, 'r')

# Create a new bag file for writing with modified frame IDs
new_bag = rosbag.Bag(bag_path + 'agent1_sim_frame_id.bag', 'w', compression=rosbag.Compression.NONE)

# Initialize the frame ID counter
frame_id_counter = 10000

# Iterate over the messages in the original bag file
for topic, msg, t in bag.read_messages():
    # Check if the message is from the camera topic
    # print(f"topic: {topic}")
    if topic == '/cam0/image_raw':
        print(f"topic: {topic}")
        # Convert the frame ID counter to a zero-padded string
        new_frame_id = str(frame_id_counter).zfill(10)
        # Modify the frame ID of the camera message
        msg.header.frame_id = new_frame_id
        print(f"new_frame_id: {new_frame_id}\n")
        # Increment the frame ID counter
        frame_id_counter += 1

    # Write the modified message to the new bag file
    new_bag.write(topic, msg, t)

# Close the bag files
bag.close()
new_bag.close()
