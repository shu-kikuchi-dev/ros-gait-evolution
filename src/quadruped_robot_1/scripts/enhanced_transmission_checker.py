#!/usr/bin/env python3

# import necessary libraries
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math
import argparse
import csv
import os

# keep curent joint position
current_joint_positions = {}
# declare joint names
joint_names = [
    'base_to_front_right_upper_leg',
    'front_right_upper_leg_to_front_right_lower_leg',

    'base_to_front_left_upper_leg',
    'front_left_upper_leg_to_front_left_lower_leg',

    'base_to_rear_right_upper_leg',
    'rear_right_upper_leg_to_rear_right_lower_leg',

    'base_to_rear_left_upper_leg',
    'rear_left_upper_leg_to_rear_left_lower_leg'
]

# declare amplitude and frequency as a parameter for sin mode
# and get parameter from gait.yaml
amplitude = rospy.get_param("/amplitude", 0.2)
frequency = rospy.get_param("/frequency", 0.5)

# arg parser setting
# e.g. for each mode
# rosrun quadruped_robot_1 enhanced_transmission_checker.py --mode sin --rate 10
# rosrun quadruped_robot_1 enhanced_transmission_checker.py --mode step --rate 5
# rosrun quadruped_robot_1 enhanced_transmission_checker.py --mode manual --manual_angles 0.5 -0.5 0.3 -0.3 0.5 -0.5 0.3 -0.3
def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['sin', 'step', 'manual'],
                        default='sin', help='Mode of operation: sin, step, manual')
    parser.add_argument('--manual_angles', nargs='*', type=float, help='joint angles for manual mode(rad)')
    parser.add_argument('--rate', type=float, default=1.0, help='control loop frequency(Hz)')
    return parser.parse_args()

# when received /joint_state topic(use msg.name and msg.position here)
def joint_state_callback(msg):
    for name, pos in zip(msg.name, msg.position):
        current_joint_positions[name] = pos # set name and position to save

# calculate target angle
def compute_target_angle(mode, t, index, manual_angles):
    if mode == 'sin':
        return amplitude * math.sin(2 * math.pi * frequency * t + index * math.pi/4)
    elif mode == 'step':
        return 1.0 if int(t) % 4 < 2 else -1.0
    elif mode == 'manual':
        return manual_angles[index] if manual_angles and index < len(manual_angles) else 0.0
    
def main():
    # define args to use arg_parser
    args = parse_args()
    # initialize node as enhanced_transmission_checker
    rospy.init_node('enhanced_transmission_checker')

    # define subscriber that receive /joint_state topic from JointStateController(gazebo_ros_control and ros_control)
    # set joint_state_callback as a callback function
    rospy.Subscriber("joint_states", JointState, joint_state_callback)

    # create publishers for each joint and save them in a list
    pubs = [rospy.Publisher(f"/{j}/command", Float64, queue_size=10) for j in joint_names]
    # wait for publishers to turn on
    rospy.sleep(2.0)

    # create transmission_log.csv in the home directory and write down the header row
    log_file_path = os.path.join(os.path.expanduser("~"), "transmission_log.csv")
    log_file = open(log_file_path, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow(['time', 'joint_name', 'desired_angle', 'actual_angle', 'error'])

    # set loop rate, start time, permissible error
    rate = rospy.Rate(args.rate)
    start_time = rospy.Time.now().to_sec()
    ERROR_THRESHOLD = 0.1

    rospy.loginfo(f"start: mode = {args.mode}, log = {log_file_path}")

    try:
        while not rospy.is_shutdown():
            # get current time ane elapsed time
            now = rospy.Time.now().to_sec()
            t = now - start_time
            # execute for each joint
            for i, (joint, pub) in enumerate(zip(joint_names, pubs)):
                # calculate target angle and publish
                target_angle = compute_target_angle(args.mode, t, i, args.manual_angles)
                pub.publish(Float64(data=target_angle))

                # get current angle
                actual_angle = current_joint_positions.get(joint, None)
                # calculate error between actual and target
                if actual_angle is not None:
                    error = abs(target_angle - actual_angle)
                    # write down to the log file
                    writer.writerow([t, joint, target_angle, actual_angle, error])
                    if error > ERROR_THRESHOLD:
                        # display warn
                        rospy.logwarn(f"[{joint}] the error is too large: {error:.3f} rad")
                else:
                    # if could not obtain current joint state
                    rospy.loginfo(f"[{joint}] failed to obtain actual joint angle")

                rospy.loginfo(f"[{joint}] cmd: {target_angle:.3f} rad")

            rate.sleep()
    # handle Ctrl+C (ROSInterruptException)
    except rospy.ROSInterruptException:
        rospy.loginfo("the process was interrupted. closing the log file.")
    finally:
        log_file.close()
        
if __name__ == '__main__':
    main()