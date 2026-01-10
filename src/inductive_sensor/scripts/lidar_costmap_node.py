#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LightweightBEV(Node):
    def __init__(self):
        super().__init__('lidar_costmap_bev')
        self.bridge = CvBridge()

        # -------------------------
        # Configuration
        # -------------------------
        self.img_size = 800           # pixels
        self.scale = 50               # pixels per meter
        self.center = self.img_size // 2

        self.publish_freq = 20.0      # Hz
        self.decay_sec = 0.5          # seconds
        self.enable_noise_removal = False
        self.noise_radius = 0.3       # meters
        self.min_neighbors = 2

        # Vehicle footprint (meters)
        self.vehicle_length = 1.2
        self.vehicle_width = 0.8

        # Static transforms to base_link (x, y, yaw)
        self.transforms = {
            '/vanjee_scan/left':  {'x': 0.0, 'y': 0.47, 'yaw': 0.698},
            '/vanjee_scan/right': {'x': 0.0, 'y': -0.47, 'yaw': -0.698}
        }

        # -------------------------
        # Data storage
        # -------------------------
        self.lidar_data = {
            '/vanjee_scan/left': None,
            '/vanjee_scan/right': None
        }

        # -------------------------
        # Subscriptions
        # -------------------------
        self.create_subscription(LaserScan, '/vanjee_scan/left',
                                 lambda msg: self.lidar_cb(msg, '/vanjee_scan/left'), 10)
        self.create_subscription(LaserScan, '/vanjee_scan/right',
                                 lambda msg: self.lidar_cb(msg, '/vanjee_scan/right'), 10)

        # -------------------------
        # Publisher
        # -------------------------
        self.publisher_ = self.create_publisher(Image, 'lidar_bev_image', 10)

        # Timer
        self.timer = self.create_timer(1.0/self.publish_freq, self.publish_frame)

    # -------------------------
    # LiDAR callback
    # -------------------------
    def lidar_cb(self, msg, topic_name):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        ranges = np.array(msg.ranges)

        valid_mask = (ranges > msg.range_min) & (ranges < msg.range_max)
        r = ranges[valid_mask]
        theta = angles[valid_mask]

        x = r * np.cos(theta)
        y = r * np.sin(theta)
        points = np.column_stack((x, y))

        self.lidar_data[topic_name] = {
            'points': points,
            'timestamp': self.get_clock().now()
        }

    # -------------------------
    # Noise removal
    # -------------------------
    def apply_noise_removal(self, points):
        if len(points) == 0:
            return points

        filtered_points = []
        for i, p1 in enumerate(points):
            diff = points - p1
            dist_sq = np.sum(diff**2, axis=1)
            neighbors = np.sum(dist_sq < self.noise_radius**2)
            if neighbors >= self.min_neighbors:
                filtered_points.append(p1)
        return np.array(filtered_points)

    # -------------------------
    # Publish BEV image
    # -------------------------
    def publish_frame(self):
        view = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        current_time = self.get_clock().now()
        active_sources = []

        for topic, data in self.lidar_data.items():
            if data is None:
                continue

            age = (current_time - data['timestamp']).nanoseconds / 1e9
            if age > self.decay_sec:
                continue

            active_sources.append(topic.split('/')[-1])
            points = data['points']

            # Transform points into base_link
            t = self.transforms[topic]
            c, s = np.cos(t['yaw']), np.sin(t['yaw'])
            rot = np.array([[c, -s],
                            [s,  c]])
            points = points @ rot.T
            points += np.array([t['x'], t['y']])

            # Optional noise removal
            if self.enable_noise_removal:
                points = self.apply_noise_removal(points)

            # Plot points
            for p in points:
                px = int(self.center - (p[1] * self.scale))  # y -> horizontal
                py = int(self.center - (p[0] * self.scale))  # x -> vertical
                if 0 <= px < self.img_size and 0 <= py < self.img_size:
                    cv2.circle(view, (px, py), 2, (0, 255, 0), -1)

        # Draw vehicle footprint
        px_len = int(self.vehicle_length * self.scale)
        px_wid = int(self.vehicle_width * self.scale)
        cv2.rectangle(
            view,
            (self.center - px_wid//2, self.center - px_len//2),
            (self.center + px_wid//2, self.center + px_len//2),
            (0, 0, 255), 2
        )

        # Add status text
        status_text = f"Active: {', '.join(active_sources) if active_sources else 'NO DATA'}"
        color = (255, 255, 255) if active_sources else (0, 0, 255)
        cv2.putText(view, status_text, (20, self.img_size - 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        # Publish
        ros_img = self.bridge.cv2_to_imgmsg(view, encoding="bgr8")
        self.publisher_.publish(ros_img)

        # Optional local display
        cv2.imshow("Operator BEV", view)
        cv2.waitKey(1)

# -------------------------
# Main
# -------------------------
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
