#!/usr/bin/python3
"""
OAK Multi-Camera H.264 Publisher Node

Manages multiple OAK-POE cameras in a single process to avoid DepthAI XLink race conditions.
Each camera connects sequentially within one process, publishes hardware-encoded H.264 streams
to separate ROS2 topics for forwarding to LiveKit.

This unified approach solves the race condition where multiple dai.Device() calls from
separate processes interfere with each other.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import depthai as dai
import time
import yaml
import os
import threading
import contextlib


class CameraHandler:
    """Handles a single OAK camera: device connection, pipeline, and frame publishing"""

    def __init__(self, camera_name, camera_ip, width, height, fps, bitrate, publisher, logger):
        self.camera_name = camera_name
        self.camera_ip = camera_ip
        self.width = width
        self.height = height
        self.fps = fps
        self.bitrate = bitrate
        self.publisher = publisher
        self.logger = logger

        # Statistics
        self.frame_count = 0
        self.last_log_time = time.time()
        self.total_bytes = 0

        # Camera resources
        self.device = None
        self.pipeline = None
        self.queue = None
        self.running = False
        self.thread = None

    def setup_camera(self, stack):
        """Initialize connection to OAK camera and configure H.264 pipeline"""
        try:
            # Connect to specific camera by IP address
            device_info = dai.DeviceInfo(self.camera_ip)
            print(device_info)
            device_info.protocol = dai.XLinkProtocol.X_LINK_TCP_IP

            self.logger.info(f'[{self.camera_name}] Connecting to {self.camera_ip}...')

            # Create device with context manager using modern API
            self.device = stack.enter_context(dai.Device(device_info))

            self.logger.info(f'[{self.camera_name}] Connected successfully')

            # Log device info
            device_id = self.device.getDeviceId()
            cameras = self.device.getConnectedCameras()
            self.logger.info(f'[{self.camera_name}] Device ID: {device_id}')
            self.logger.info(f'[{self.camera_name}] Cameras: {[c.name for c in cameras]}')

            # Create pipeline with device context
            self.pipeline = dai.Pipeline(self.device)

            # Configure RGB camera using new Camera node API
            cam = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)

            # Request video output at specified resolution in NV12 format (required for H.264 encoder)
            videoOutput = cam.requestOutput(
                (self.width, self.height),
                type=dai.ImgFrame.Type.NV12,
                fps=self.fps
            )

            # Configure H.264 encoder (runs on camera hardware!)
            encoder = self.pipeline.create(dai.node.VideoEncoder)
            encoder.setDefaultProfilePreset(
                self.fps,
                dai.VideoEncoderProperties.Profile.H264_MAIN
            )
            encoder.setBitrate(self.bitrate)
            encoder.setKeyframeFrequency(self.fps * 2)  # Keyframe every 2 seconds

            # Link camera video output to encoder
            videoOutput.link(encoder.input)

            # Create output queue for H.264 encoded stream with maxSize=1 for minimum latency
            h264Queue = encoder.bitstream.createOutputQueue(maxSize=1, blocking=False)

            # Start pipeline
            self.pipeline.start()

            # Store the queue
            self.queue = h264Queue

            self.logger.info(f'[{self.camera_name}] Pipeline started on {self.camera_ip}')
            return True

        except Exception as e:
            self.logger.error(f'[{self.camera_name}] Failed to setup camera {self.camera_ip}: {e}')
            return False

    def is_keyframe(self, data):
        """Check if H.264 data contains a keyframe (IDR frame)"""
        # Look for NAL unit type 5 (IDR frame)
        data_bytes = bytes(data)
        for i in range(len(data_bytes) - 5):
            # Check for NAL start codes: 0x00 0x00 0x01 or 0x00 0x00 0x00 0x01
            if data_bytes[i:i+3] == b'\x00\x00\x01':
                nal_type = data_bytes[i+3] & 0x1F
                if nal_type == 5:  # IDR frame
                    return True
            elif data_bytes[i:i+4] == b'\x00\x00\x00\x01':
                nal_type = data_bytes[i+4] & 0x1F
                if nal_type == 5:  # IDR frame
                    return True
        return False

    def process_frames(self, clock):
        """Read H.264 packets from camera and publish to ROS (runs in thread)"""
        self.running = True

        while self.running and self.queue:
            try:
                # Non-blocking get - try to get latest frame
                frame = self.queue.get()

                if frame is None:
                    # No frame available yet
                    time.sleep(0.001)  # Small sleep to avoid busy loop
                    continue

                # Get H.264 encoded data
                h264_data = frame.getData()

                if len(h264_data) == 0:
                    continue

                # Get timestamp
                now = clock.now()

                # Check if this is a keyframe
                is_keyframe = self.is_keyframe(h264_data)

                # Create and publish CompressedImage message
                msg = CompressedImage()
                msg.header = Header()
                msg.header.stamp = now.to_msg()
                msg.header.frame_id = self.camera_name
                msg.format = 'h264'
                msg.data = bytes(h264_data)

                self.publisher.publish(msg)

                # Update statistics
                self.frame_count += 1
                self.total_bytes += len(h264_data)

                # Log statistics every 5 seconds
                current_time = time.time()
                if current_time - self.last_log_time >= 5.0:
                    elapsed = current_time - self.last_log_time
                    actual_fps = self.frame_count / elapsed
                    avg_size = self.total_bytes / self.frame_count if self.frame_count > 0 else 0
                    bitrate_mbps = (self.total_bytes * 8) / (elapsed * 1_000_000)

                    self.logger.info(
                        f'[{self.camera_name}] {self.frame_count} frames, '
                        f'{actual_fps:.1f} fps, avg={int(avg_size)} bytes/frame, '
                        f'{bitrate_mbps:.2f} Mbps'
                    )

                    if is_keyframe:
                        self.logger.info(f'[{self.camera_name}] Keyframe: {len(h264_data)} bytes')

                    # Reset statistics
                    self.frame_count = 0
                    self.total_bytes = 0
                    self.last_log_time = current_time

            except RuntimeError as e:
                # Queue closed or pipeline stopped
                if "closed" in str(e).lower():
                    self.logger.warn(f'[{self.camera_name}] Queue closed, stopping')
                    break
                else:
                    self.logger.error(f'[{self.camera_name}] Runtime error: {e}')
            except Exception as e:
                self.logger.error(f'[{self.camera_name}] Error reading frame: {e}')

    def start_processing(self, clock):
        """Start the frame processing thread"""
        self.thread = threading.Thread(target=self.process_frames, args=(clock,), daemon=True)
        self.thread.start()
        self.logger.info(f'[{self.camera_name}] Frame processing thread started')

    def stop(self):
        """Stop camera and cleanup resources"""
        self.logger.info(f'[{self.camera_name}] Stopping...')
        self.running = False

        if self.thread:
            self.thread.join(timeout=2.0)

        # Device will be closed by ExitStack context manager


class OAKMultiCameraH264Node(Node):
    """Unified ROS2 node that manages multiple OAK cameras in a single process"""

    def __init__(self):
        super().__init__('oak_multicam_h264_node')

        # Load camera configuration
        package_share_dir = get_package_share_directory('luxonis_cam_pipeline')
        config_path = os.path.join(package_share_dir, 'config', 'streams.yaml')

        print("Loading config from:", config_path)
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        streams = config['/**']['ros__parameters']['streams']
        camera_names = streams['names']
        camera_ips = streams['camera_ips']
        widths = streams['widths']
        heights = streams['heights']
        fps_list = streams['fps']
        bitrates = streams['bitrates']

        # QoS profile for low-latency streaming (BEST_EFFORT to match Go bridge expectation)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create ExitStack for device resource management
        self.stack = contextlib.ExitStack()

        # Create camera handlers
        self.cameras = []

        for i, camera_name in enumerate(camera_names):
            camera_ip = camera_ips[i]
            width = widths[i]
            height = heights[i]
            fps = fps_list[i]
            bitrate = bitrates[i]

            self.get_logger().info(f'Setting up camera: {camera_name} at {camera_ip} ({width}x{height} @ {fps}fps)')

            # Create publisher for this camera
            publisher = self.create_publisher(
                CompressedImage,
                f'/{camera_name}/rgb/h264',
                qos
            )

            # Create camera handler
            camera = CameraHandler(
                camera_name, camera_ip, width, height, fps, bitrate,
                publisher, self.get_logger()
            )

            # Setup camera with ExitStack (sequential connection to avoid race conditions)
            if camera.setup_camera(self.stack):
                self.cameras.append(camera)
                # Delay between camera setups to ensure clean XLink initialization
                # DepthAI XLink protocol needs time to fully stabilize connections
                # 10 seconds required to avoid IP address confusion between devices
                time.sleep(10)
            else:
                self.get_logger().error(f'Failed to setup {camera_name}, skipping')

        self.get_logger().info(f'Successfully initialized {len(self.cameras)}/{len(camera_names)} cameras')

        # Start frame processing for all cameras
        for camera in self.cameras:
            camera.start_processing(self.get_clock())

        self.get_logger().info('All camera processing threads started')

    def destroy_node(self):
        """Cleanup all camera resources"""
        self.get_logger().info('Shutting down all cameras...')

        for camera in self.cameras:
            camera.stop()

        # Close all devices via ExitStack
        self.stack.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OAKMultiCameraH264Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()