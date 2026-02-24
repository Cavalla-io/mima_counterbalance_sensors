import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from ament_index_python.packages import get_package_share_directory
import cv2 # Re-adding for placeholder
import numpy as np # Re-adding for placeholder
import torch
from ultralytics import YOLO
from PIL import Image # Keep for now, might be useful for cv_bridge or transforms

# Import our custom message
from threat_detector_msgs.msg import ThreatAlert

# For YOLOv5, you might need to define class names
# COCO_CLASSES = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', ...]

class ThreatDetectionNode(Node):

    def __init__(self):
        super().__init__('threat_detection_node')
        self.subscription = self.create_subscription(
            Image,
            '/oak/rgb/image',  # Topic from luxonis_cam_pipeline
            self.image_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(ThreatAlert, 'threat_alerts', 10)
        self.bridge = CvBridge()
        self.get_logger().info('Threat Detection Node has been started.')

        # --- PyTorch Model Initialization ---
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.get_logger().info(f"Using device: {self.device}.")

        # Load YOLOv8 model
        # Assuming yolov8m.pt is in the models directory
        model_pt_path = os.path.join(get_package_share_directory('threat_detector'), 'models', 'yolov8m.pt')
        
        try:
            # Load model using ultralytics YOLO class
            self.model = YOLO(model_pt_path) # This will load the local .pt file
            self.model.to(self.device)
            self.model.eval() # Set model to evaluation mode
            self.get_logger().info(f"PyTorch YOLOv8m model loaded successfully on {self.device}.")
        except Exception as e:
            self.get_logger().error(f"Failed to load PyTorch YOLOv8m model: {e}")
            self.model = None # Ensure model is None if loading fails

        self.conf_threshold = 0.25 # Confidence threshold for detections
        # NMS is handled internally by YOLOv8 model's predict method, so nms_threshold is not directly used here
        
        # Define your class names for YOLOv8 (e.g., COCO classes)
        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
        # --- End PyTorch Model Initialization ---


    def image_callback(self, msg):
        # self.get_logger().info('Received image frame.') # Comment out for less verbose output
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Check for empty/invalid image
        if cv_image.size == 0:
            self.get_logger().warn("Received an empty image, skipping processing.")
            return

        (h, w) = cv_image.shape[:2]
        threat_detected = False
        object_name = "no_threat"
        confidence = 0.0
        bbox_xywh = []
        alert_message = "No threat detected."

        if self.model:
            # --- Real PyTorch Model Inference (YOLOv8 Example) ---
            # The YOLO model's predict method handles preprocessing, inference, and NMS
            # It expects a numpy array (BGR or RGB) or PIL Image
            results = self.model.predict(source=cv_image, conf=self.conf_threshold, verbose=False)
            
            # Post-process detections
            boxes, confidences, class_ids = self._postprocess(cv_image, results)

            if len(boxes) > 0:
                # For simplicity, let's just take the first detected object as the "threat"
                # In a real scenario, you'd have logic to prioritize threats
                box = boxes[0]
                conf = confidences[0]
                class_id = class_ids[0]

                object_name = self.classes[class_id] if self.classes and class_id < len(self.classes) else "unknown"
                confidence = float(conf)
                bbox_xywh = [int(box[0]), int(box[1]), int(box[2]), int(box[3])] # x, y, w, h

                threat_detected = True
                alert_message = f"Threat detected: {object_name} with confidence {confidence:.2f} at {bbox_xywh}"
                self.get_logger().warn(alert_message)
            # --- End Real PyTorch Model Inference ---
        else:
            self.get_logger().warn("PyTorch model not loaded. Using placeholder detection.")
            # --- Placeholder for your Computer Vision Model (Simulated) ---
            # This is the previous red detection logic, kept as a fallback if model not loaded
            # Note: cv2 and numpy are still needed for this placeholder
            hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])
            mask1 = cv2.inRange(hsv_image, lower_red, upper_red)

            lower_red = np.array([170, 100, 100])
            upper_red = np.array([180, 255, 255])
            mask2 = cv2.inRange(hsv_image, lower_red, upper_red)

            red_mask = mask1 + mask2
            red_pixels = np.sum(red_mask > 0)
            total_pixels = cv_image.shape[0] * cv_image.shape[1]

            if red_pixels / total_pixels > 0.05:  # If more than 5% of pixels are red
                threat_detected = True
                object_name = "potential_danger_zone"
                confidence = red_pixels / total_pixels
                # Simulate a bounding box for the entire image if red is widespread
                bbox_xywh = [0, 0, cv_image.shape[1], cv_image.shape[0]]
                alert_message = f"Simulated threat: High concentration of red detected ({confidence:.2f})."
                self.get_logger().warn(alert_message)
            # --- End of Placeholder ---

        if threat_detected:
            alert_msg = ThreatAlert()
            alert_msg.header.stamp = self.get_clock().now().to_msg()
            alert_msg.header.frame_id = msg.header.frame_id
            alert_msg.object_name = object_name
            alert_msg.confidence = confidence
            alert_msg.bbox_xywh = bbox_xywh
            alert_msg.message = alert_message
            self.publisher_.publish(alert_msg)
            self.get_logger().info(f'Published Threat Alert: {alert_message}')

    def _postprocess(self, original_cv_image, results):
        # This post-processing logic handles the Results object from ultralytics YOLOv8.
        # The predict method already performs NMS and scaling.
        
        boxes = []
        confidences = []
        class_ids = []

        # Iterate through the results (one Results object per image in batch)
        for r in results:
            # r.boxes contains detection bounding boxes and scores
            # r.boxes.xywhn: normalized xywh boxes
            # r.boxes.xyxy: absolute xyxy boxes
            # r.boxes.conf: confidence scores
            # r.boxes.cls: class IDs

            if r.boxes is not None:
                for i in range(len(r.boxes)):
                    conf = r.boxes.conf[i].item()
                    cls = r.boxes.cls[i].item()
                    
                    # Filter by confidence threshold
                    if conf > self.conf_threshold:
                        # Get bounding box in xyxy format (absolute coordinates)
                        x1, y1, x2, y2 = r.boxes.xyxy[i].tolist()

                        # Convert to (x, y, w, h) format
                        box_x = int(x1)
                        box_y = int(y1)
                        box_w = int(x2 - x1)
                        box_h = int(y2 - y1)

                        boxes.append([box_x, box_y, box_w, box_h])
                        confidences.append(conf)
                        class_ids.append(int(cls))
        
        return boxes, confidences, class_ids


def main(args=None):
    rclpy.init(args=args)
    threat_detection_node = ThreatDetectionNode()
    rclpy.spin(threat_detection_node)
    threat_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
