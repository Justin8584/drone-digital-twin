#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleLocalPosition

class PositionMonitor(Node):
    def __init__(self):
        super().__init__('position_monitor')
        self.sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.callback,
            10)
        self.count = 0
        
    def callback(self, msg):
        self.count += 1
        if self.count % 10 == 0:  # Print every 10th message
            print(f"Position: X={msg.x:.2f}m, Y={msg.y:.2f}m, Z={msg.z:.2f}m, Heading={msg.heading:.2f}rad")

def main():
    rclpy.init()
    node = PositionMonitor()
    print("Monitoring position... (Ctrl+C to stop)")
    rclpy.spin(node)

if __name__ == '__main__':
    main()
