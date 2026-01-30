import rclpy
from rclpy.node import Node
import socket
import struct
import yaml
import os
import json
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import String

# If you use the custom message:
# from pull_wire_encoder.msg import ForkStatus

class ForkPublisher(Node):
    def __init__(self):
        super().__init__('fork_publisher')
        
        # 1. CONFIGURATION
        self.interface = "can1"
        self.node_id = 5
        self.cob_id = 0x180 + self.node_id
        
        # 2. LOAD CONFIG
        self.cfg = self.load_config()
        self.cal = self.cfg['calibration']
        self.safety = self.cfg['safety']
        
        # 3. SETUP PUBLISHER
        self.publisher_ = self.create_publisher(String, 'fork_position', 10)
        
        # 4. SETUP CAN SOCKET
        self.sock = self.setup_socket()

        # 5. START LOOP (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info(f"Fork Node Started. Listening on {self.interface}...")

    def load_config(self):
        try:
            package_share_directory = get_package_share_directory('pull_wire_encoder')
            config_path = os.path.join(package_share_directory, 'config', 'encoder-config.yaml')
            
            with open(config_path, 'r') as f:
                data = yaml.safe_load(f)
            return data['encoder_config']
        except Exception as e:
            self.get_logger().error(f"Failed to load config: {e}")
            return {}

    def setup_socket(self):
        try:
            sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((self.interface,))
            sock.setblocking(False) 
            return sock
        except Exception as e:
            self.get_logger().error(f"Failed to bind CAN socket: {e}")
            return None

    def calculate_height(self, raw_ticks):
        net_ticks = raw_ticks - self.cal['zero_point_ticks']
        height = net_ticks * self.cal['resolution_mm_per_tick'] * self.cal['direction_scalar']
        return float(round(height, 2))

    def determine_status(self, height):
        # PRIORITY 1: Safety/Software Limits (User Configured)
        # If we violate these, we return 3 immediately, ignoring everything else.
        
        # Check Max Allowed (Software Ceiling)
        if height >= self.safety['max_allowed_height_mm']:
            return 3 # SAFETY_VIOLATION (Out of Allowed Limits)

        # Check Min Allowed (Software Floor / Slack Cable Error)
        if height <= self.safety['min_allowed_height_mm']:
            return 3 # SAFETY_VIOLATION (Negative/Slack Error)

        # PRIORITY 2: Mechanical Limits (Physical Stops)
        # We only check these if we are INSIDE the allowed safety zone.
        
        # Check Upper Mechanical Limit (Physical Top)
        if height >= (self.cal['mechanical_max_height_mm'] - 5.0):
            return 2 # UPPER_MECHANICAL_LIMIT

        # Check Lower Mechanical Limit (Physical Bottom/Ground)
        if height <= 5.0:
            return 1 # LOWER_MECHANICAL_LIMIT
            
        # PRIORITY 3: Safe
        return 0 # SAFE

    def timer_callback(self):
        if not self.sock: return

        try:
            while True:
                frame = self.sock.recv(16)
                can_id, _, data = struct.unpack("=IB3x8s", frame)
                can_id &= 0x1FFFFFFF

                if can_id == self.cob_id:
                    raw_ticks = struct.unpack('<I', data[:4])[0]
                    
                    height = self.calculate_height(raw_ticks)
                    status_code = self.determine_status(height)

                    # Create Payload
                    msg_payload = {
                        "height": height,
                        "status": status_code,
                        "desc": ["SAFE", "LOWER_MECH", "UPPER_MECH", "SAFETY_VIOLATION"][status_code]
                    }
                    
                    msg = String()
                    msg.data = json.dumps(msg_payload)

                    self.publisher_.publish(msg)
                    
        except BlockingIOError:
            pass 
        except Exception as e:
            self.get_logger().warn(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ForkPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()