#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math
import argparse
import csv
import os

# === グローバル変数 ===
current_joint_positions = {}
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

def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mode', choices=['sin', 'step', 'manual'], default='sin')
    parser.add_argument('--manual_angles', nargs='*', type=float, help='manualモード用の関節角度 (rad)')
    parser.add_argument('--rate', type=float, default=1.0, help='制御ループの周波数 [Hz]')
    return parser.parse_args()

def joint_state_callback(msg):
    for name, pos in zip(msg.name, msg.position):
        current_joint_positions[name] = pos

def compute_target_angle(mode, t, index, manual_angles):
    if mode == 'sin':
        return math.sin(t + index * math.pi / 4)
    elif mode == 'step':
        return 1.0 if int(t) % 4 < 2 else -1.0
    elif mode == 'manual':
        return manual_angles[index] if manual_angles and index < len(manual_angles) else 0.0

def main():
    args = parse_args()
    rospy.init_node('enhanced_transmission_checker')

    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    pubs = [rospy.Publisher(f"/{j}/command", Float64, queue_size=10) for j in joint_names]
    rospy.sleep(2.0)

    log_file_path = os.path.join(os.path.expanduser("~"), "transmission_log.csv")
    log_file = open(log_file_path, 'w', newline='')
    writer = csv.writer(log_file)
    writer.writerow(['time', 'joint_name', 'desired_angle', 'actual_angle', 'error'])

    rate = rospy.Rate(args.rate)
    start_time = rospy.Time.now().to_sec()
    ERROR_THRESHOLD = 0.1

    rospy.loginfo(f"開始：モード = {args.mode}, ログ = {log_file_path}")

    try:
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            t = now - start_time

            for i, (joint, pub) in enumerate(zip(joint_names, pubs)):
                target_angle = compute_target_angle(args.mode, t, i, args.manual_angles)
                pub.publish(Float64(data=target_angle))

                actual_angle = current_joint_positions.get(joint, None)
                if actual_angle is not None:
                    error = abs(target_angle - actual_angle)
                    writer.writerow([t, joint, target_angle, actual_angle, error])
                    if error > ERROR_THRESHOLD:
                        rospy.logwarn(f"[{joint}] 誤差が大きすぎます: {error:.3f} rad")
                else:
                    rospy.logwarn(f"[{joint}] 実角度を取得できません")

                rospy.loginfo(f"[{joint}] cmd: {target_angle:.3f} rad")

            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("中断されました。ログファイルを閉じます。")
    finally:
        log_file.close()

if __name__ == '__main__':
    main()
