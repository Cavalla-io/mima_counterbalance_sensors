import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import can

class SteeringPublisher(Node):
    def __init__(self):
        super().__init__('steering_publisher')
        self.publisher_ = self.create_publisher(Float32, 'steering_angle', 10)
        
        # Setup can iface
        filters = [{"can_id": 0x383, "can_mask": 0x7FF, "extended": False}]
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan', can_filters=filters)
        
        # Check for messages every 0.01s (100Hz)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # timeout=0 makes this non-blocking
        msg = self.bus.recv(timeout=0)
        
        if msg is not None:
            raw_value = int.from_bytes(msg.data[0:2], byteorder='little', signed=True)
            angle = raw_value * 0.01
            
            ros_msg = Float32()
            ros_msg.data = angle
            self.publisher_.publish(ros_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SteeringPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
