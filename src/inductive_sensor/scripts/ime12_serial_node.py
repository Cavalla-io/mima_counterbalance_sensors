#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8


class IME12SerialNode(Node):
    def __init__(self):
        super().__init__('ime12_serial_node')

        self.pub = self.create_publisher(
            Int8,
            'inductive_sensors',
            10
        )

        self.port = '/dev/ttyACM0'
        self.baud = 115200
        self.ser = None
        self.warned = False

        self.timer = self.create_timer(0.1, self.loop)

    def try_connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            self.get_logger().info(f'Connected to {self.port}')
            self.warned = False
        except serial.SerialException:
            if not self.warned:
                self.get_logger().warn(
                    f'No serial device on {self.port} — waiting for sensor readings'
                )
                self.warned = True
            self.ser = None

    def loop(self):
        if self.ser is None:
            self.try_connect()
            return

        if self.ser.in_waiting:
            line = self.ser.readline().decode(errors='ignore').strip()
            try:
                s2, s1 = map(int, line.split(','))
                # Convert to single state: 0,0->0 | 0,1->1 | 1,0->2 | 1,1->3
                """
                0,0 -> Fully Loaded -> 0
                1,1 -> Unloaded -> 3 
                0,1 -> Left -> 1
                1,0 -> Right -> 2
                """
                state = (s1 << 1) | s2
                msg = Int8()
                msg.data = state
                self.pub.publish(msg)
            except ValueError:
                pass


def main():
    rclpy.init()
    rclpy.spin(IME12SerialNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()