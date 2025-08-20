#!/usr/bin/env python3
"""
object_localizer.py
A ROS2 node that detects a red cube in the RGB image, finds its pixel location,
and computes its 3D position using the depth image and camera intrinsics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class ObjectLocalizer(Node):
    def __init__(self):
        # Initialize ROS2 node with the name "object_localizer"
        super().__init__('object_localizer')

        # --- ROS2 Subscribers ---
        # RGB camera feed
        self.rgb_sub = self.create_subscription(Image, '/UG1/rgb', self.rgb_callback, 10)
        # Depth camera feed
        self.depth_sub = self.create_subscription(Image, '/UG1/depth', self.depth_callback, 10)
        # Camera intrinsics
        self.cam_info_sub = self.create_subscription(CameraInfo, '/UG1/camera_info', self.caminfo_callback, 10)

        # Bridge for converting ROS2 Image → OpenCV image
        self.bridge = CvBridge()

        # Store latest depth frame
        self.depth_image = None

        # Camera intrinsics (focal lengths and optical center)
        self.fx = self.fy = self.cx = self.cy = None

    def caminfo_callback(self, msg: CameraInfo):
        """
        Callback for CameraInfo:
        - Extract intrinsic parameters (fx, fy, cx, cy).
        - These are used for projecting 2D pixels into 3D space.
        """
        self.fx = msg.k[0]  # focal length in x
        self.fy = msg.k[4]  # focal length in y
        self.cx = msg.k[2]  # principal point x
        self.cy = msg.k[5]  # principal point y
        self.get_logger().info("Camera intrinsics loaded.")

        # Unsubscribe after receiving intrinsics once
        self.destroy_subscription(self.cam_info_sub)

    def depth_callback(self, msg: Image):
        """
        Callback for Depth Image:
        - Convert ROS2 depth image into OpenCV format.
        - Depth values are in meters.
        """
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def rgb_callback(self, msg: Image):
        """
        Callback for RGB Image:
        1. Detect the red object in the RGB frame.
        2. Find the centroid pixel (u, v).
        3. Look up depth at (u, v).
        4. Compute 3D (X, Y, Z) coordinates using intrinsics.
        """
        if self.depth_image is None or self.fx is None:
            # Wait until both depth image and camera intrinsics are available
            return

        # Convert ROS2 Image → OpenCV BGR image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert to HSV (better for color segmentation)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red ranges in HSV (wraps around hue scale)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        # Threshold image for red regions
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Find contours of detected red areas
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Select the largest red object (assumed cube)
            largest = max(contours, key=cv2.contourArea)
            M = cv2.moments(largest)

            if M['m00'] > 0:
                # Compute centroid (pixel coordinates)
                u = int(M['m10'] / M['m00'])
                v = int(M['m01'] / M['m00'])

                # Depth at pixel (u, v)
                z = float(self.depth_image[v, u])

                if z > 0:  # Valid depth
                    # Back-project to 3D camera coordinates
                    X = (u - self.cx) * z / self.fx
                    Y = (v - self.cy) * z / self.fy
                    Z = z

                    # Log cube position
                    self.get_logger().info(f"Cube position: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")

                # Draw detection on RGB image for visualization
                cv2.circle(frame, (u, v), 5, (0, 255, 0), -1)
                cv2.putText(frame, f"({X:.2f}, {Y:.2f}, {Z:.2f})",
                            (u + 10, v), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 255), 2)

        # Debug visualization windows (RGB + mask)
        cv2.imshow("RGB", frame)
        cv2.imshow("Mask", mask)
        cv2.waitKey(1)


def main(args=None):
    """
    Entry point:
    - Initialize ROS2.
    - Start ObjectLocalizer node.
    - Keep spinning until shutdown.
    """
    rclpy.init(args=args)
    node = ObjectLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
