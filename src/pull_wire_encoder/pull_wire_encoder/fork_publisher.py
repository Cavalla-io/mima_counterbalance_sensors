import rclpy
from rclpy.node import Node
import socket
import struct
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32

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
        self.publisher_ = self.create_publisher(Float32, 'fork_position', 10)
        
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

                    msg = Float32()
                    msg.data = height

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
