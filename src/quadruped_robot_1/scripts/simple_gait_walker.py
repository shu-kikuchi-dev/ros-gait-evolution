import rospy
from std_msgs.msg import Float64
import math
import time

def main():
    rospy.init_node('simple_gait_controller')

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
        pub = respy.Publisher(topic_name, Float64, queue_size=10)
        pubs.append(pub)
    
    rospy.sleep(1) # waiting for publisher preparation

    rate = rospy.Rate(10) # 10 Hz
    t = 0.0

    while not rospy.is_shutdown():
        # move legs with simple sin waves
        for i, pub in enumerate(pubs):
            angle = 0.5 * math.sin(t + i * math.pi / 4)
            pub.publish(Float64(angle))
        t += 0.1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()