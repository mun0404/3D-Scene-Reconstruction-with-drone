#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class BestFrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver_node')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(Image, '/hires_small_color', self.listener_callback, 10)

        self.image_publisher = self.create_publisher(Image, '/saved_image', 10)
        self.orb = cv2.ORB_create(nfeatures=1000)

        # FLANN Matcher setup for binary descriptors
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH, table_number=6, key_size=12, multi_probe_level=1)
        search_params = dict(checks=50)
        self.matcher = cv2.FlannBasedMatcher(index_params, search_params)

        # Frame Storage
        self.reference_frame = None
        self.reference_kp = None
        self.reference_des = None

        # Save folder
        self.save_folder = '/home/munawwar/Final Colmap/best_frames'
        os.makedirs(self.save_folder, exist_ok=True)

        # Threshold
        self.match_threshold_ratio = 0.35

        # Image counter
        self.num_images = 0

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # First frame
        if self.reference_frame is None:
            self.save_frame(frame)
            self.reference_frame = gray_frame
            self.reference_kp, self.reference_des = self.orb.detectAndCompute(self.reference_frame, None)
            return

        # Detect features
        kp, des = self.orb.detectAndCompute(gray_frame, None)

        if des is None or self.reference_des is None:
            self.get_logger().warning('Descriptors not found, skipping frame.')
            return

        # Match descriptors
        try:
            des = np.uint8(des)
            reference_des = np.uint8(self.reference_des)

            matches = self.matcher.match(reference_des, des)
            matches = sorted(matches, key=lambda x: x.distance)
            good_matches = [m for m in matches if m.distance < 30]
            num_good_matches = len(good_matches)

            self.get_logger().info(f'Number of good matches : {num_good_matches}')

            if num_good_matches < self.match_threshold_ratio * len(self.reference_kp):
                self.save_frame(frame)
                self.reference_frame = gray_frame
                self.reference_kp = kp
                self.reference_des = des

        except Exception as e:
            self.get_logger().error(f'Error in matching: {e}')

    def save_frame(self, frame):
        
        # Publish saved frame (converted back to ROS2 Image)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.image_publisher.publish(msg)

        filename = os.path.join(self.save_folder, f'frame_{self.num_images:04d}.png')
        cv2.imwrite(filename, frame)
        self.get_logger().info(f'Saved frame: {filename}')
        self.num_images += 1

def main(args=None):
    rclpy.init(args=args)
    node = BestFrameSaver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
