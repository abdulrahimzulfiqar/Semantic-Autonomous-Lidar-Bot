#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import math
import time
from tf2_ros import TransformBroadcaster

class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.get_logger().info("--- ARDUINO BRIDGE STARTED (DEBUG MODE) ---")
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB_ARDUINO')
        self.declare_parameter('baud', 115200)
        
        # Robot Physical Parameters
        # CALIBRATED 2026-02-09: 4505 ticks / 10 revs = 450.5
        self.ticks_per_rev = 450.5 
        self.wheel_radius = 0.0325
        self.wheel_base = 0.169 # TUNED 2026-02-11: Decreased further to 0.169 to fix remaining 5deg under-spin
        self.circumference = 2 * math.pi * self.wheel_radius
        self.meters_per_tick = self.circumference / self.ticks_per_rev

        # Serial
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return

        # Pub/Sub
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom_raw', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu_raw', 10)
        self.tf_broadcaster = TransformBroadcaster(self) # Initialize TF Broadcaster
        
        # Odom Vars
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.initialized = False

        self.create_timer(0.02, self.update) # 50Hz

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        
        # 1. Calculate target velocities for each wheel in m/s
        v_left = linear - (angular * self.wheel_base / 2.0)
        v_right = linear + (angular * self.wheel_base / 2.0)
        
        # 2. Convert m/s to ticks/sec
        # ticks_per_meter = 1.0 / meters_per_tick
        ticks_per_meter = 1.0 / self.meters_per_tick
        
        target_left_ticks_sec = v_left * ticks_per_meter
        target_right_ticks_sec = v_right * ticks_per_meter
        
        # 3. Send command to Arduino
        cmd = f"M,{target_left_ticks_sec:.2f},{target_right_ticks_sec:.2f}\n"
        
        # DEBUG: Log command to verify bridge is sending velocities
        self.get_logger().info(f"Sending Target Ticks/Sec: {cmd.strip()}")
        self.ser.write(cmd.encode())

    def update(self):
        last_valid_data = None
        
        # Drain the buffer (Read EVERYTHING waiting)
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                if line.startswith('E,'):
                    data = line.split(',')
                    if len(data) == 9:
                        last_valid_data = data
            except Exception as e:
                pass
        
        # Only process the NEWEST packet (if any)
        if last_valid_data:
            self.process_data(last_valid_data)

    def process_data(self, data):
        try:
            # Removed software inversion, arduino_bridge.ino handles it now!
            left_ticks = int(data[1])
            right_ticks = int(data[2])
            
            # Debug log ENABLED for troubleshooting
            self.get_logger().info(f"TICKS: Left={left_ticks}, Right={right_ticks}")

            current_time = self.get_clock().now()
            
            if not self.initialized:
                self.prev_left_ticks = left_ticks
                self.prev_right_ticks = right_ticks
                self.initialized = True
                self.get_logger().info("Sensors Initialized! Zeroing encoders.")
                return

            # Odom Math
            d_left = (left_ticks - self.prev_left_ticks) * self.meters_per_tick
            d_right = (right_ticks - self.prev_right_ticks) * self.meters_per_tick
            self.prev_left_ticks = left_ticks
            self.prev_right_ticks = right_ticks
            
            d_dist = (d_right + d_left) / 2.0
            d_th = (d_right - d_left) / self.wheel_base
            
            self.th += d_th
            self.x += d_dist * math.cos(self.th)
            self.y += d_dist * math.sin(self.th)

            # Publish Odom
            odom = Odometry()
            odom.header.stamp = current_time.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            q = self.euler_to_quaternion(0, 0, self.th)
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            self.pub_odom.publish(odom)

            # Broadcast TF (Direct Encoder Visualization)
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_footprint"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

            # Publish Imu
            imu = Imu()
            imu.header.stamp = current_time.to_msg()
            imu.header.frame_id = "imu_link"
            imu.linear_acceleration.x = float(data[3])
            imu.linear_acceleration.y = float(data[4])
            imu.linear_acceleration.z = float(data[5])
            imu.angular_velocity.x = float(data[6])
            imu.angular_velocity.y = float(data[7])
            imu.angular_velocity.z = float(data[8])
            self.pub_imu.publish(imu)

        except Exception as e:
            self.get_logger().error(f"Parse Error: {e}")

    def euler_to_quaternion(self, r, p, y):
        qz = math.sin(y/2) * math.cos(p/2) * math.cos(r/2) - math.cos(y/2) * math.sin(p/2) * math.sin(r/2)
        qw = math.cos(y/2) * math.cos(p/2) * math.cos(r/2) + math.sin(y/2) * math.sin(p/2) * math.sin(r/2)
        return [0.0, 0.0, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
