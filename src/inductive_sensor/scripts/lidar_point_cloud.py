#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from sensor_msgs_py import point_cloud2
from cv_bridge import CvBridge
import cv2
import numpy as np

import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm




class MultiLidarBEV(Node):
    def __init__(self):
        super().__init__('vanjee_bev_node')
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # --- Configuration ---
        self.img_size = 800
        self.scale = 50 
        self.center = self.img_size // 2
        
        # --- Height Filter (Meters) ---
        self.min_height = -0.1  # Adjust based on floor level
        self.max_height = 0.1   # 10cm limit
        
        # --- Data Storage ---
        self.lidar_sources = {
            # '/vanjee_points/left': {'points': np.empty((0, 2)), 'last_seen': 0.0},
            '/vanjee_points/right': {'points': np.empty((0, 2)), 'last_seen': 0.0}
        }

        # --- Subscriptions ---
        for topic in self.lidar_sources.keys():
            self.create_subscription(
                PointCloud2, 
                topic, 
                lambda msg, t=topic: self.points_cb(msg, t), 
                10)
        
        self.publisher_ = self.create_publisher(Image, 'lidar_bev_image', 10)
        self.timer = self.create_timer(0.05, self.publish_frame)

    def points_cb(self, msg, topic_name):
        transform = self.tf_buffer.lookup_transform(
            "base_link",                     # target frame
            msg.header.frame_id,       # source frame (lidar)
            rclpy.time.Time()
        )

        cloud_map = tf2_sm.do_transform_cloud(msg, transform)
        xy = []

        for p in point_cloud2.read_points(
            cloud_map,
            field_names=("x", "y", "z"),
            skip_nans=True
        ):
            x, y, z = p
            if self.min_height <= z <= self.max_height:
                xy.append((x, y))

        if not xy:
            return

        self.lidar_sources[topic_name]['points'] = np.asarray(xy, dtype=np.float32)
        self.lidar_sources[topic_name]['last_seen'] = (
            self.get_clock().now().nanoseconds * 1e-9
        )
        
    def publish_frame(self):
        view = np.zeros((self.img_size, self.img_size, 3), dtype=np.uint8)
        current_time = self.get_clock().now().nanoseconds / 1e9
        active_topics = []

        for topic, data in self.lidar_sources.items():
            if current_time - data['last_seen'] > 0.5:
                continue
            
            active_topics.append(topic.split('/')[-1])
            pts = data['points']

            if pts.size > 0:
                # Optimized vectorized conversion to pixels
                # x -> vertical (py), y -> horizontal (px)
                pxs = (self.center - (pts[:, 1] * self.scale)).astype(int)
                pys = (self.center - (pts[:, 0] * self.scale)).astype(int)

                # Filter points within image bounds
                valid = (pxs >= 0) & (pxs < self.img_size) & (pys >= 0) & (pys < self.img_size)
                
                # Draw points (Green for left, Cyan for right)
                color = (0, 255, 0) if 'left' in topic else (255, 255, 0)
                
                # Use a loop for drawing pixels or a fast mask update
                for i in range(len(pxs)):
                    if valid[i]:
                        cv2.circle(view, (pxs[i], pys[i]), 1, color, -1)

        # Draw Ego Vehicle
        cv2.rectangle(view, (self.center-10, self.center-15), 
                      (self.center+10, self.center+15), (0,0,255), 2)

        # HUD
        status = f"Active: {', '.join(active_topics) if active_topics else 'OFFLINE'}"
        cv2.putText(view, status, (20, self.img_size - 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        ros_img = self.bridge.cv2_to_imgmsg(view, encoding="bgr8")
        self.publisher_.publish(ros_img)

        cv2.imshow("Dual Vanjee BEV", view)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MultiLidarBEV()
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