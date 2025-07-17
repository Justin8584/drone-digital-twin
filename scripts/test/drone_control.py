#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, VehicleStatus
import sys

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # Publisher for commands
        self.cmd_pub = self.create_publisher(
            VehicleCommand, 
            '/fmu/in/vehicle_command', 
            10)
        
        # Subscriber for status
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v1',
            self.status_callback,
            10)
        
        self.is_armed = False
        
    def status_callback(self, msg):
        self.is_armed = (msg.arming_state == 2)
        
    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        
        self.cmd_pub.publish(msg)
        
    def arm(self):
        print("Sending ARM command...")
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0  # 1.0 = arm
        )
        
    def disarm(self):
        print("Sending DISARM command...")
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0  # 0.0 = disarm
        )
        
    def takeoff(self, altitude=5.0):
        if not self.is_armed:
            print("Vehicle not armed! Arm first.")
            return
            
        print(f"Sending TAKEOFF command to {altitude}m...")
        self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param1=altitude
        )

def main():
    rclpy.init()
    node = DroneController()
    
    if len(sys.argv) < 2:
        print("Usage: python3 drone_control.py [arm|disarm|takeoff]")
        return
        
    command = sys.argv[1].lower()
    
    # Give node time to receive status
    rclpy.spin_once(node, timeout_sec=0.5)
    
    if command == "arm":
        node.arm()
    elif command == "disarm":
        node.disarm()
    elif command == "takeoff":
        node.takeoff()
    else:
        print(f"Unknown command: {command}")
        
    # Spin for a bit to send the command
    rclpy.spin_once(node, timeout_sec=1.0)
    
    # Check status
    rclpy.spin_once(node, timeout_sec=0.5)
    print(f"Armed status: {'ARMED' if node.is_armed else 'DISARMED'}")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
