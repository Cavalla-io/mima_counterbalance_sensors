#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class LightweightBEV(Node):
    def __init__(self):
        super().__init__('LidarCostMap')
        self.bridge = CvBridge()
        
        # --- Configuration ---
        self.img_size = 800
        self.scale = 50 
        self.center = self.img_size // 2
        
        # Parameters for features requested
        self.publish_freq = 20.0  # Hz
        self.decay_sec = 0.5      # How long to keep lidar data valid
        self.enable_noise_removal = False
        self.noise_radius = 0.3    # Meters
        self.min_neighbors = 2
        
        # --- Data Storage ---
        # Stores {'points': np.array, 'timestamp': float}
        self.lidar_data = {
            '/vanjee_scan/left': None,
            '/vanjee_scan/right': None
        }

        # --- Subscriptions & Publishers ---
        self.create_subscription(LaserScan, '/vanjee_scan/left', 
            lambda msg: self.lidar_cb(msg, '/vanjee_scan/left'), 10)
        self.create_subscription(LaserScan, '/vanjee_scan/right', 
            lambda msg: self.lidar_cb(msg, '/vanjee_scan/right'), 10)
        
        # Publisher for the processed BEV image
        self.publisher_ = self.create_publisher(Image, 'lidar_bev_image', 10)
        
        # Timer for fixed frequency publishing
        self.timer = self.create_timer(1.0/self.publish_freq, self.publish_frame)

    def lidar_cb(self, msg, topic_name):
        """Asynchronously save points and timestamp."""
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)
        
        # Filter valid ranges
        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        r = ranges[valid_mask]
        theta = angles[valid_mask]

        # Convert to Cartesian (meters)
        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points = np.column_stack((x, y))

        self.lidar_data[topic_name] = {
            'points': points,
            'timestamp': self.get_clock().now()
        }

    def apply_noise_removal(self, points):
        """Simple Radius Outlier Removal using Euclidean distance."""
        if len(points) == 0: return points
        
        filtered_points = []
        for i, p1 in enumerate(points):
            # Check distances to all other points
            diff = points - p1
            dist_sq = np.sum(diff**2, axis=1)
            neighbors = np.sum(dist_sq < self.noise_radius**2)
            
            if neighbors >= self.min_neighbors:
                filtered_points.append(p1)
                
        return np.array(filtered_points)

    def publish_frame(self):
        # Create blank canvas
        view = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        current_time = self.get_clock().now()
        active_sources = []
        
        for topic, data in self.lidar_data.items():
            if data is None:
                continue
            
            # 1. Check Decay/Timestamp
            age = (current_time - data['timestamp']).nanoseconds / 1e9
            if age > self.decay_sec:
                continue # Skip old data
            
            active_sources.append(topic.split('/')[-1])
            points = data['points']

            # 2. Optional Noise Removal
            if self.enable_noise_removal:
                points = self.apply_noise_removal(points)

            # 3. Plotting
            for p in points:
                # To Pixels (y is horizontal in image, x is vertical)
                px = int(self.center - (p[1] * self.scale))
                py = int(self.center - (p[0] * self.scale))
                
                if 0 <= px < self.img_size and 0 <= py < self.img_size:
                    cv2.circle(view, (px, py), 2, (0, 255, 0), -1)

        # Draw Ego Vehicle
        cv2.rectangle(view, (self.center-10, self.center-15), 
                      (self.center+10, self.center+15), (0,0,255), 2)

        # 4. Add Status Message at the bottom
        status_text = f"Active: {', '.join(active_sources) if active_sources else 'NO DATA'}"
        color = (255, 255, 255) if active_sources else (0, 0, 255)
        cv2.putText(view, status_text, (20, self.img_size - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # 5. Publish to Topic
        ros_img = self.bridge.cv2_to_imgmsg(view, encoding="bgr8")
        self.publisher_.publish(ros_img)

        # Optional: Local Display
        cv2.imshow("Operator BEV", view)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = LightweightBEV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()