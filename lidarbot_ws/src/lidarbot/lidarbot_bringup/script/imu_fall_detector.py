#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import math

class IMUFallDetector(Node):
    def __init__(self):
        super().__init__('imu_fall_detector')
        
        # Subscribe to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu_raw',
            self.imu_callback,
            10
        )
        
        # Publisher for web app alerts
        self.alert_publisher = self.create_publisher(
            String,
            '/emergency_alerts',
            10
        )
        
        # Thresholds (Tweak these depending on physical testing)
        self.FREEFALL_THRESHOLD = 3.0  # m/s^2 (Standard gravity is ~9.81)
        self.IMPACT_THRESHOLD = 18.0   # Sudden sharp spike
        self.TILT_THRESHOLD = 0.5      # Z-axis gravity drops too low (tipped over)

        self.get_logger().info('IMU Fall & Collision Detector Started. Monitoring /imu_raw...')

    def imu_callback(self, msg):
        # Extract Linear Acceleration
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Calculate total acceleration vector magnitude
        total_accel = math.sqrt(ax**2 + ay**2 + az**2)
        
        collision_detected = False
        fall_detected = False
        alert_msg = ""

        # 1. Check for severe Impact / Collision (High Spike)
        if total_accel > self.IMPACT_THRESHOLD:
            collision_detected = True
            alert_msg = f"COLLISION WARNING! High G-Force spike detected: {total_accel:.2f} m/s^2"
            
        # 2. Check for freefall / tip-over (Total Accel drops significantly or Z gravity shifts)
        # Note: If wheel chair tips over completely, az will drop close to 0 and ax/ay will read ~9.8
        elif total_accel < self.FREEFALL_THRESHOLD:
            fall_detected = True
            alert_msg = f"FALL DETECTED! Freefall condition met: {total_accel:.2f} m/s^2"
        elif abs(az) < self.TILT_THRESHOLD and total_accel > 8.0:
            fall_detected = True
            alert_msg = f"TIP-OVER WARNING! Wheelchair is no longer upright. Z-Axis: {az:.2f}"

        # If any anomaly is detected, publish it to the React Dashboard
        if collision_detected or fall_detected:
            self.get_logger().warn(alert_msg)
            
            ros_msg = String()
            ros_msg.data = alert_msg
            self.alert_publisher.publish(ros_msg)


def main(args=None):
    rclpy.init(args=args)
    detector = IMUFallDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        pass
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
