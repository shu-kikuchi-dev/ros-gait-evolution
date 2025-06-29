#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import time

def main():
    rospy.init_node('simple_transmission_checker')

    # define publishers for each joints
    joint_names = [
        'base_to_front_right_upper_leg',
        'front_right_upper_leg_to_front_right_lower_leg',

        'base_to_front_left_upper_leg',
        'front_left_upper_leg_to_front_left_lower_leg',

        'base_to_rear_right_upper_leg',
        'rear_right_upper_leg_to_rear_right_lower_leg',

        'base_to_rear_left_upper_leg',
        'rear_left_upper_leg_to_rear_left_lower_leg',
    ]

    pubs = []
    for joint in joint_names:
        topic_name = f'/{joint}/command'
        pub = rospy.Publisher(topic_name, Float64, queue_size=10)
        pubs.append(pub)
    
    rospy.loginfo("Waiting for publishers to register...")
    rospy.sleep(3) # waiting for publisher preparation

    start_time = rospy.Time.now().to_sec()
    rospy.loginfo(f'Start time* {start_time}')

    rate = rospy.Rate(1) # 1 Hz

    rospy.loginfo("Starting gait control loop.")

    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_time

            # move legs with simple sin waves
            for i, pub in enumerate(pubs):
                angle = 1.0 * math.sin(t + i * math.pi / 4)
                pub.publish(Float64(data=angle))
                rospy.loginfo(f'Joint {joint_names[i]}: angle = {angle:.3f}')
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down gait controller.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass