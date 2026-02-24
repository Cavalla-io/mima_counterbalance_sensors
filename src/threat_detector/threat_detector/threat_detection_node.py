import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

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

        # --- CV Model Initialization ---
        # You will need to download your YOLOv5n model (e.g., yolov8n.onnx)
        # and place it in a 'models' directory within your package.
        # Example: /ros_ws/src/threat_detector/models/yolov8n.onnx

        model_name = 'yolov5s6_640_ti_lite_best.optimized.onnx' # Replace with your model file name
        model_dir = os.path.join(get_package_share_directory('threat_detector'), 'models')
        model_path = os.path.join(model_dir, model_name)

        self.net = None
        if os.path.exists(model_path):
            try:
                self.net = cv2.dnn.readNet(model_path)
                self.get_logger().info(f"CV model '{model_name}' loaded successfully from {model_path}.")
            except Exception as e:
                self.get_logger().error(f"Failed to load CV model '{model_name}': {e}")
        else:
            self.get_logger().warn(f"CV model '{model_name}' not found at {model_path}. Using placeholder detection.")
        
        # Set preferred backend and target for OpenCV DNN
        # For Jetson, use CUDA backend and CUDA target for GPU acceleration
        if self.net:
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
            self.get_logger().info("OpenCV DNN backend set to CUDA for GPU acceleration.")

        self.input_width = 640 # YOLOv5 input size
        self.input_height = 640 # YOLOv5 input size
        self.conf_threshold = 0.25 # Confidence threshold for detections
        self.nms_threshold = 0.45 # Non-maximum suppression threshold

        # Define your class names for YOLOv5 (e.g., COCO classes)
        self.classes = ['person', 'bicycle', 'car', 'motorcycle', 'airplane', 'bus', 'train', 'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter', 'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe', 'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball', 'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle', 'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange', 'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'couch', 'potted plant', 'bed', 'dining table', 'toilet', 'tv', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave', 'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear', 'hair drier', 'toothbrush']
        # --- End CV Model Initialization ---


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

        if self.net:
            # --- Real CV Model Inference (YOLOv5 Example) ---
            # Preprocess the image for the model
            blob = cv2.dnn.blobFromImage(cv_image, 1/255.0, (self.input_width, self.input_height), swapRB=True, crop=False)
            self.net.setInput(blob)
            
            # Run inference
            outputs = self.net.forward(self.net.getUnconnectedOutLayersNames())
            
            # Post-process detections
            boxes, confidences, class_ids = self._postprocess(cv_image, outputs)

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
            # --- End Real CV Model Inference ---
        else:
            # --- Placeholder for your Computer Vision Model (Simulated) ---
            # This is the previous red detection logic, kept as a fallback if model not loaded
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

    def _postprocess(self, frame, outputs):
        # This post-processing logic is specific to the YOLOv5 model output.
        # It assumes that the output format is [cx, cy, w, h, obj_conf, class_scores...].
        
        h, w = frame.shape[:2]
        boxes = []
        confidences = []
        class_ids = []

        # Iterate through the outputs
        for output in outputs:
            output_tensor = np.squeeze(output) # Remove batch dimension

            # Determine if transpose is needed based on typical YOLO output shapes
            # If the number of rows is significantly smaller than the number of columns,
            # it's likely (channels, detections) and needs transposing to (detections, channels).
            # A typical YOLO output has (num_detections, 5 + num_classes).
            # 5 + num_classes is usually around 80-100, while num_detections is in the thousands.
            if output_tensor.shape[0] < output_tensor.shape[1]:
                detections = output_tensor.transpose(1, 0)
            else:
                detections = output_tensor

            for det in detections:
                # Ensure detection has enough elements for unpacking
                if len(det) < 6: # Need at least cx, cy, w, h, obj_conf, and one class score
                    continue

                object_conf = det[4]
                class_scores = det[5:]

                # Ensure class_scores is not empty
                if class_scores.size == 0:
                    continue

                # Manually find the index of the maximum score to bypass potential np.argmax issues
                max_score = -1.0
                class_id = -1
                for i in range(class_scores.size):
                    if class_scores[i] > max_score:
                        max_score = class_scores[i]
                        class_id = i
                
                # If no valid class_id was found (e.g., all scores were negative or class_scores was empty)
                if class_id == -1:
                    continue

                class_conf = max_score # The confidence of the highest scoring class
                confidence = float(object_conf * class_conf)

                if confidence > self.conf_threshold:
                    center_x, center_y, width, height = det[0:4] * np.array([w, h, w, h])
                    
                    x = int(center_x - width / 2)
                    y = int(center_y - height / 2)
                    
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(confidence) # Use the combined confidence
                    class_ids.append(class_id)

        # Apply Non-Maximum Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, self.nms_threshold)
        # Check if any detections remain after NMS
        if indices.size > 0:
            indices = indices.flatten()
            boxes = [boxes[i] for i in indices]
            confidences = [confidences[i] for i in indices]
            class_ids = [class_ids[i] for i in indices]
        else:
            boxes, confidences, class_ids = [], [], []

        return boxes, confidences, class_ids


def main(args=None):
    rclpy.init(args=args)
    threat_detection_node = ThreatDetectionNode()
    rclpy.spin(threat_detection_node)
    threat_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
