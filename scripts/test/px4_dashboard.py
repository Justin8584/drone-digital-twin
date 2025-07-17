#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    VehicleStatus, 
    VehicleLocalPosition,
    BatteryStatus,
    VehicleGlobalPosition,
    SensorCombined
)
import os
import time

class PX4Dashboard(Node):
    def __init__(self):
        super().__init__('px4_dashboard')
        
        # Create subscribers
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1', 
            self.status_cb, 10)
            
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', 
            self.local_pos_cb, 10)
            
        self.global_pos_sub = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', 
            self.global_pos_cb, 10)
            
        self.battery_sub = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status_v1', 
            self.battery_cb, 10)
            
        self.sensor_sub = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined', 
            self.sensor_cb, 10)
        
        # Initialize data
        self.armed = False
        self.nav_state = 0
        self.local_pos = {'x': 0, 'y': 0, 'z': 0}
        self.velocity = {'vx': 0, 'vy': 0, 'vz': 0}
        self.global_pos = {'lat': 0, 'lon': 0, 'alt': 0}
        self.battery_pct = 100
        self.accel = {'x': 0, 'y': 0, 'z': 0}
        self.last_update = time.time()
        
    def status_cb(self, msg):
        self.armed = (msg.arming_state == 2)
        self.nav_state = msg.nav_state
        self.last_update = time.time()
        
    def local_pos_cb(self, msg):
        self.local_pos = {'x': msg.x, 'y': msg.y, 'z': msg.z}
        self.velocity = {'vx': msg.vx, 'vy': msg.vy, 'vz': msg.vz}
        
    def global_pos_cb(self, msg):
        self.global_pos = {
            'lat': msg.lat, 
            'lon': msg.lon, 
            'alt': msg.alt
        }
        
    def battery_cb(self, msg):
        self.battery_pct = msg.remaining * 100
        
    def sensor_cb(self, msg):
        # Accelerometer data (m/s²)
        self.accel = {
            'x': msg.accelerometer_m_s2[0],
            'y': msg.accelerometer_m_s2[1], 
            'z': msg.accelerometer_m_s2[2]
        }
        
    def display(self):
        os.system('clear')
        print("╔══════════════════════ PX4 Dashboard ══════════════════════╗")
        print(f"║ Status: {'ARMED ⚡' if self.armed else 'DISARMED':<15} Battery: {self.battery_pct:>5.1f}% ║")
        print("╠═══════════════════════════════════════════════════════════╣")
        print(f"║ Local Position:  X: {self.local_pos['x']:>7.2f}m  Y: {self.local_pos['y']:>7.2f}m  Z: {self.local_pos['z']:>7.2f}m ║")
        print(f"║ Velocity:       Vx: {self.velocity['vx']:>7.2f}   Vy: {self.velocity['vy']:>7.2f}   Vz: {self.velocity['vz']:>7.2f} ║")
        print(f"║ GPS Position:  Lat: {self.global_pos['lat']:.6f}  Lon: {self.global_pos['lon']:.6f} ║")
        print(f"║ Altitude:      {self.global_pos['alt']:.2f}m                                    ║")
        print(f"║ Acceleration:   Ax: {self.accel['x']:>6.2f}   Ay: {self.accel['y']:>6.2f}   Az: {self.accel['z']:>6.2f} ║")
        print("╠═══════════════════════════════════════════════════════════╣")
        print(f"║ Last Update: {time.time() - self.last_update:.1f}s ago                              ║")
        print("╚═══════════════════════════════════════════════════════════╝")
        print("\nPress Ctrl+C to exit")

def main():
    rclpy.init()
    node = PX4Dashboard()
    
    print("Starting PX4 Dashboard...")
    print("Waiting for data...")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            node.display()
            time.sleep(0.1)  # 10Hz update
    except KeyboardInterrupt:
        print("\nShutting down...")
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()
