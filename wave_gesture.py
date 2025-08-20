#!/usr/bin/env python3
"""
wave_gesture.py
A simple ROS2 node that makes the robot perform a waving motion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import signal
import sys


class WaveNode(Node):
    def __init__(self):
        # Initialize the ROS2 node with the name "wave_node"
        super().__init__('wave_node')

        # Publisher to send joint commands to the robot
        self.publisher_ = self.create_publisher(JointState, '/UG1/joint_command', 10)

        # Define target joint positions for the pre-wave pose (arm raised)
        self.pre_wave_pose = {
            'right_shoulder_pitch_joint': 0.9,
            'right_shoulder_roll_joint': -1.5,
            'right_shoulder_yaw_joint': -1.8,
            'right_elbow_joint': 0.0
        }

        # Define the neutral pose (resting position)
        self.neutral_pose = {
            'right_shoulder_pitch_joint': 0.0,
            'right_shoulder_roll_joint': 0.0,
            'right_shoulder_yaw_joint': 0.0,
            'right_elbow_joint': 0.0
        }

        # Store the robot's current joint positions (start in neutral pose)
        self.current_positions = self.neutral_pose.copy()

        # Flags for controlling motion sequence
        self.in_pre_wave = False   # Whether robot reached pre-wave pose
        self.counter = 0           # Counter for sinusoidal waving motion

        # Create a timer to repeatedly call the control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Register a signal handler to safely stop on Ctrl+C
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def timer_callback(self):
        """
        Main control loop:
        - First, move arm smoothly into pre-wave pose.
        - Once in position, perform waving by oscillating the elbow joint.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        if not self.in_pre_wave:
            # Move smoothly towards the pre-wave pose
            msg.name = list(self.pre_wave_pose.keys())
            positions = []
            reached = True  # Flag to check if target is reached

            for joint, target in self.pre_wave_pose.items():
                current = self.current_positions[joint]
                diff = target - current
                step = 0.03  # Increment per iteration for smooth motion

                # Adjust current joint position gradually
                if abs(diff) > step:
                    current += step if diff > 0 else -step
                    reached = False
                else:
                    current = target

                # Update stored joint position
                self.current_positions[joint] = current
                positions.append(current)

            msg.position = positions

            # Once pose is reached, switch to waving motion
            if reached:
                self.in_pre_wave = True

        else:
            # Perform waving motion (oscillate elbow joint back and forth)
            msg.name = list(self.pre_wave_pose.keys())
            positions = list(self.current_positions.values())

            # Find index of elbow joint
            elbow_index = msg.name.index('right_elbow_joint')

            # Apply sinusoidal motion to elbow joint
            positions[elbow_index] += 0.3 * math.sin(self.counter * 0.1)

            msg.position = positions
            self.counter += 1  # Increase counter for oscillation step

        # Publish the joint state command
        self.publisher_.publish(msg)

    def return_to_neutral(self):
        """
        Send the robot back to the neutral resting pose.
        """
        self.get_logger().info("Returning to neutral pose...")

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.neutral_pose.keys())
        msg.position = list(self.neutral_pose.values())

        self.publisher_.publish(msg)
        self.get_logger().info("Neutral pose command sent.")

    def shutdown_handler(self, sig, frame):
        """
        Safely handle shutdown:
        - Return robot to neutral pose.
        - Shutdown ROS2.
        - Exit program.
        """
        self.return_to_neutral()
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    """
    Entry point for the node:
    - Initialize ROS2.
    - Start WaveNode.
    - Spin until shutdown.
    """
    rclpy.init(args=args)
    node = WaveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
