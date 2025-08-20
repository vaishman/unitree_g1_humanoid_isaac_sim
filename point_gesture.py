#!/usr/bin/env python3
"""
point_gesture.py
A ROS2 node that makes the humanoid robot perform a pointing gesture.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import signal, sys

# Define the relevant arm joints for positioning
ARM_JOINTS = [
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]

# Define hand joints (for fingers and thumb)
HAND_JOINTS = [
    'right_hand_index_0_joint', 'right_hand_index_1_joint',
    'right_hand_middle_0_joint', 'right_hand_middle_1_joint',
    'right_hand_thumb_0_joint', 'right_hand_thumb_1_joint', 'right_hand_thumb_2_joint',
]

# Full ordering of joints (arm first, then hand)
ORDER = ARM_JOINTS + HAND_JOINTS


class PointGesture(Node):
    def __init__(self):
        # Initialize ROS2 node with the name "point_node"
        super().__init__('point_node')

        # Publisher for sending joint commands
        self.pub = self.create_publisher(JointState, '/UG1/joint_command', 10)

        # Target joint configuration for a pointing gesture:
        # - Arm raised forward with straight elbow
        # - Index finger extended
        # - Other fingers curled
        # - Thumb curled inward slightly for natural posture
        self.target = {
            'right_shoulder_pitch_joint': -1.0,   # arm forward
            'right_shoulder_roll_joint': -0.15,  # slight outward roll
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 1.0,            # straight elbow
            'right_wrist_roll_joint': 0.0,
            'right_wrist_pitch_joint': 0.0,
            'right_wrist_yaw_joint': 0.0,

            # index extended (straight)
            'right_hand_index_0_joint': 0.0,
            'right_hand_index_1_joint': 0.0,

            # middle finger curled
            'right_hand_middle_0_joint': 0.9,
            'right_hand_middle_1_joint': 0.9,

            # thumb curled (small yaw offset for realism)
            'right_hand_thumb_0_joint': 0.2,
            'right_hand_thumb_1_joint': -0.5,
            'right_hand_thumb_2_joint': 0.9,
        }

        # Initialize all controlled joints to neutral (0.0 radians)
        self.state = {name: 0.0 for name in self.target.keys()}

        # Timer for control loop (runs at 10 Hz)
        self.timer = self.create_timer(0.1, self.on_timer)

        # Register Ctrl+C signal handler for safe shutdown
        signal.signal(signal.SIGINT, self.on_sigint)

    def on_timer(self):
        """
        Main control loop:
        - Gradually move joints from current state to the target pointing pose.
        - Each step is limited for smooth motion.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        step = 0.06  # max change per joint per cycle
        names, pos = [], []

        # Iterate in defined order for consistent control
        for name in ORDER:
            if name not in self.target:
                continue  # skip unused joints

            cur = self.state[name]
            tgt = self.target[name]
            diff = tgt - cur

            # Adjust joint towards target in small increments
            if abs(diff) > step:
                cur += step if diff > 0 else -step
            else:
                cur = tgt

            self.state[name] = cur
            names.append(name)
            pos.append(cur)

        msg.name = names
        msg.position = pos

        # Publish updated joint state command
        self.pub.publish(msg)

    def on_sigint(self, *_):
        """
        Handle Ctrl+C:
        - Reset all controlled joints to neutral (0.0).
        - Shutdown ROS2 cleanly.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.state.keys())
        msg.position = [0.0] * len(msg.name)

        # Send neutral command before shutdown
        self.pub.publish(msg)

        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    """
    Entry point:
    - Initialize ROS2.
    - Start the PointGesture node.
    - Keep running until shutdown.
    """
    rclpy.init(args=args)
    node = PointGesture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
