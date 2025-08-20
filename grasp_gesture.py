#!/usr/bin/env python3
"""
grasp_gesture.py
A ROS2 node that makes the humanoid robot perform a grasping motion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import signal, sys

# Define the relevant arm joints
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

# Order of joints for consistent control
ORDER = ARM_JOINTS + HAND_JOINTS


class GraspGesture(Node):
    def __init__(self):
        # Initialize ROS2 node with the name "grasp_node"
        super().__init__('grasp_node')

        # Publisher to send joint commands to the robot
        self.pub = self.create_publisher(JointState, '/UG1/joint_command', 10)

        # Target configuration for a grasping gesture:
        # - Arm slightly forward, elbow bent
        # - All fingers curled inward
        # - Thumb curled naturally without twisting (small base angle, distal curl)
        self.target = {
            'right_shoulder_pitch_joint': -0.4,   # arm slightly forward
            'right_shoulder_roll_joint': 0.0,
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 0.5,             # slight elbow bend
            'right_wrist_roll_joint': 0.0,
            'right_wrist_pitch_joint': 0.0,
            'right_wrist_yaw_joint': 0.0,

            # index and middle finger curled
            'right_hand_index_0_joint': 0.9,
            'right_hand_index_1_joint': 0.9,
            'right_hand_middle_0_joint': 0.9,
            'right_hand_middle_1_joint': 0.9,

            # thumb curled realistically
            'right_hand_thumb_0_joint': 0.2,   # small base rotation
            'right_hand_thumb_1_joint': -0.5,  # curl joint
            'right_hand_thumb_2_joint': 0.9,   # curl joint
        }

        # Initialize joint states to neutral (0.0)
        self.state = {name: 0.0 for name in self.target.keys()}

        # Timer for control loop (runs at 10 Hz)
        self.timer = self.create_timer(0.1, self.on_timer)

        # Handle Ctrl+C for safe shutdown
        signal.signal(signal.SIGINT, self.on_sigint)

    def on_timer(self):
        """
        Main control loop:
        - Smoothly move all joints from neutral to the target grasping pose.
        - Each joint is incremented gradually to avoid sudden jumps.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        step = 0.06  # maximum change per joint per cycle
        names, pos = [], []

        for name in ORDER:
            if name not in self.target:
                continue  # skip unused joints

            cur = self.state[name]
            tgt = self.target[name]
            diff = tgt - cur

            # Move joint incrementally toward target
            if abs(diff) > step:
                cur += step if diff > 0 else -step
            else:
                cur = tgt

            # Store updated joint position
            self.state[name] = cur
            names.append(name)
            pos.append(cur)

        msg.name = names
        msg.position = pos

        # Publish joint state command
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
    - Start the GraspGesture node.
    - Keep running until shutdown.
    """
    rclpy.init(args=args)
    node = GraspGesture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
