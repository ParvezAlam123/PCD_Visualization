#!/usr/bin/env python

import rospy
import json
from geometry_msgs.msg import PoseStamped


file_path = "/media/parvez/Expansion/LocTest/ndt_pose/ndt_pose_data.json"

def callback(data):
    # Convert the ROS message to a dictionary
    data_dict = {
        'header': {
            'seq': data.header.seq,
            'stamp': {
                'secs': data.header.stamp.secs,
                'nsecs': data.header.stamp.nsecs
            },
            'frame_id': data.header.frame_id
        },
        'pose': {
            'position': {
                'x': data.pose.position.x,
                'y': data.pose.position.y,
                'z': data.pose.position.z
            },
            'orientation': {
                'x': data.pose.orientation.x,
                'y': data.pose.orientation.y,
                'z': data.pose.orientation.z,
                'w': data.pose.orientation.w
            }
        }
    }

    # Append the data to a JSON file
    with open(file_path, 'a') as f:
        json.dump(data_dict, f)
        f.write('\n')

def listener():
    rospy.init_node('json_saver', anonymous=True)
    rospy.Subscriber("/ndt_pose", PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
