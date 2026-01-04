import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


import json
import numpy as np




class TurnOverlayGround(Node):
    def __init__(self):
        super().__init__('turn_overlay_ground')

        self.get_logger().info("TurnOverlayGround node loaded")

        self.bridge = CvBridge()
        self.turn_val = 0.0

        # Assumptions
        self.cam_height = 1.8  # meters (Camera is at [0, 1.0, 0] in world frame)
        self.cam_pitch = -math.radians(80)  # downward tilt

        # Subscriptions
        self.create_subscription(Float32, '/turn_cmd', self.turn_callback, 10)
        # self.create_subscription(Image, '/front_low/rgb/h264', self.image_callback, 10)
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.create_subscription(
            CompressedImage,
            '/front_low/rgb/h264',
            self.image_callback,
            qos
        )

        # Assuming '/oak/stereo/camera_info' provides the intrinsics for the /oak/rgb/image_rect topic
        # self.create_subscription(CameraInfo, '/oak/stereo/camera_info', self.camera_info_callback, 10)
        
        with open('/home/cavallatestbench/mima_counterbalance_sensors/src/luxonis_cams/luxonis_cams/system_calibration_dump.json', 'r') as f:
            cam_json = json.load(f)

        # Get the first cameraData (or pick the camera index you want)
        camera_data = cam_json['front_cam']['raw_eeprom']['cameraData'][0][1]

        # Extract intrinsic matrix
        self.intrinsic_matrix = np.array(camera_data['intrinsicMatrix'])
        self.K = self.intrinsic_matrix  # Camera intrinsics
        

        self.publisher = self.create_publisher(Image, '/turn_overlay/image', 10)

    def turn_callback(self, msg):
        self.turn_val = max(-1.0, min(msg.data, 1.0))
        # self.get_logger().info(f"[info] Updated turn value: {self.turn_val:.2f}")

    def camera_info_callback(self, msg: CameraInfo):
        if self.K is None:
            self.K = np.array(msg.k).reshape((3,3))
            self.get_logger().info(f"[info] Camera intrinsics received: fx={self.K[0,0]:.1f}, fy={self.K[1,1]:.1f}, cx={self.K[0,2]:.1f}, cy={self.K[1,2]:.1f}")

    def world_to_image(self, x, y, z,h):
        if self.K is None:
            return None
        
        
        # 1. Define the rotation matrix for pitch (Rx)
        cos_p = math.cos(self.cam_pitch)
        sin_p = math.sin(self.cam_pitch)
        Rx = np.array([[1, 0, 0],
                       [0, cos_p, -sin_p],
                       [0, sin_p, cos_p]])
                       
        # 2. Translate P_w to Camera Origin (P_w - T_c)
        #    T_c = [0, cam_height, 0]. 
        #    The y coordinate must be shifted by subtracting the camera height.
        point_translated = np.array([x, y - self.cam_height, z]).T
        
        # 3. Rotate to Camera Frame P_c = Rx * P_translated
        cam_point = Rx @ point_translated
        
        Xc, Yc, Zc = cam_point
        
        if Zc <= 0:
            # Point is behind the camera
            return None
            
        # 4. Project using K (Pinhole Model)
        u = int(self.K[0,0] * (Xc/Zc) + self.K[0,2]) 
        v = int(self.K[1,1] * (-Yc/Zc) + self.K[1,2]) + h//2
        return (u,v)

    def draw_ground_path(self, frame):
        h, w = frame.shape[:2]
        
        points = []
        angle_deg = self.turn_val * 90
        vehicle_lenght = 1 
        wheel_base =  1.2
        if self.turn_val > 0.05 or self.turn_val  < -0.05:  
            radius = vehicle_lenght / math.sin(math.radians(abs(angle_deg))) * 1.1
        else:
            radius = -1
        n_points = 10000
        
        for t in np.linspace(0,3, n_points):
            # distance along the path
            if radius < t + vehicle_lenght and radius != -1:
                t = radius - vehicle_lenght
                
            z = t # forward distance
            if radius == -1:
                x = 0
            else: 
                # x = (-math.sqrt(radius**2- t**2) + radius) * (angle_deg / abs(angle_deg))  # lateral
                x = (math.sqrt(radius**2 - vehicle_lenght**2) - math.sqrt(radius**2 - (t+vehicle_lenght)**2) ) * (angle_deg / abs(angle_deg))  # lateral
            # x = 0  # lateral
            y = 0.0  # constant height (ground)
            
            img_pt_left = self.world_to_image(x-0.1 + wheel_base/2, y, z,h)
            img_pt_right = self.world_to_image(x+0.1-+ wheel_base/2, y, z,h)
            
            
            # print("First projected point:", self.world_to_image(0,0,0.5))
            
            # Check if point is within image bounds (optional, but good practice)
            # if img_pt_left and 0 <= img_pt_left[0] < w and 0 <= img_pt_left[1] < h:
            points.append(img_pt_left)
            # if img_pt_right and 0 <= img_pt_right[0] < w and 0 <= img_pt_right[1] < h:
            points.append(img_pt_right)

        point_base = [] 
        # Robot base projection
        for t in np.linspace(0, 3, n_points):
            z = t # forward distance
            y = 0.0  # constant height (ground)
            
            img_pt_left = self.world_to_image(wheel_base/2, y, z,h)
            img_pt_right = self.world_to_image(-wheel_base/2, y, z,h)
            
            
            # print("First projected point:", self.world_to_image(0,0,0.5))
            
            # Check if point is within image bounds (optional, but good practice)
            if img_pt_left and 0 <= img_pt_left[0] < w and 0 <= img_pt_left[1] < h:
                point_base.append(img_pt_left)
            if img_pt_right and 0 <= img_pt_right[0] < w and 0 <= img_pt_right[1] < h:
                point_base.append(img_pt_right)
        
            
        # Draw the points
        for pt in points:
            # Draw smaller circles to show the line better
            cv2.circle(frame, pt, 1, (0, 255, 255), -1)
        
        for pt in point_base:
            # Draw smaller circles to show the line better
            cv2.circle(frame, pt, 1, (255, 0, 255), -1) 
            
        # Draw lines between the projected points to form a smooth path
        overlay = frame.copy()

        for i in range(len(points) - 1):
            cv2.line(overlay, points[i], points[i+1], (0, 255, 255), 3)

        alpha = 0.4  # transparency factor, lower = more transparent
        frame = cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0)

        cv2.putText(frame, f"Turn: {angle_deg:.1f} deg",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        return frame
    def image_callback(self, msg: CompressedImage):
        print("got image")
        try:
            # Convert compressed image to OpenCV
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is None:
                raise ValueError("cv2.imdecode returned None")

        except Exception as e:
            self.get_logger().error(f"[ERROR] Failed to decode compressed image: {e}")
            return

        if self.K is not None:
            frame = self.draw_ground_path(frame)
        else:
            self.get_logger().warn("[WARN] No camera intrinsics yet, skipping overlay")

        overlay_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TurnOverlayGround()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
