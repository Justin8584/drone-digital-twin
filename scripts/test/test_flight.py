#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition
)
import time

class TestFlight(Node):
    def __init__(self):
        super().__init__('test_flight')
        
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.status_cb, 10)
            
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.pos_cb, 10)
        
        self.armed = False
        self.altitude = 0.0
        
    def status_cb(self, msg):
        self.armed = (msg.arming_state == 2)
        
    def pos_cb(self, msg):
        self.altitude = -msg.z  # NED frame, so negative z is up
        
    def send_command(self, command, param1=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)
        
    def run_test_sequence(self):
        print("=== PX4 Test Flight Sequence ===")
        
        # Wait for connection
        print("Waiting for PX4 connection...")
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.5)
            if hasattr(self, 'armed'):
                break
        
        # Step 1: Arm
        print("\n1. Arming vehicle...")
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        time.sleep(2)
        rclpy.spin_once(self, timeout_sec=0.5)
        print(f"   Armed: {'YES' if self.armed else 'NO'}")
        
        if not self.armed:
            print("   Failed to arm! Check QGroundControl for errors.")
            return
            
        # Step 2: Takeoff
        print("\n2. Taking off to 5m...")
        self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, 5.0)
        
        # Monitor altitude
        print("   Monitoring altitude...")
        for i in range(50):  # 5 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            print(f"   Altitude: {self.altitude:.2f}m", end='\r')
            if self.altitude > 4.5:
                print(f"\n   Reached target altitude: {self.altitude:.2f}m")
                break
                
        # Step 3: Hold
        print("\n3. Holding position for 3 seconds...")
        time.sleep(3)
        
        # Step 4: Land
        print("\n4. Landing...")
        self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
        # Monitor landing
        for i in range(100):  # 10 seconds
            rclpy.spin_once(self, timeout_sec=0.1)
            print(f"   Altitude: {self.altitude:.2f}m", end='\r')
            if self.altitude < 0.5:
                print(f"\n   Landed!")
                break
                
        # Step 5: Disarm
        print("\n5. Disarming...")
        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
        time.sleep(1)
        
        print("\n=== Test Complete! ===")

def main():
    rclpy.init()
    node = TestFlight()
    
    try:
        node.run_test_sequence()
    except KeyboardInterrupt:
        print("\nTest interrupted!")
        # Emergency disarm
        node.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
