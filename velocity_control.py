#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint
from geometry_msgs.msg import Twist
import math

class DroneVelocityControl(Node):
    """Send velocity commands to drone"""
    
    def __init__(self):
        super().__init__('drone_velocity_control')
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        
        # Timer for control loop (50ms)
        self.timer = self.create_timer(0.05, self.timer_callback)
        
        # Velocity command parameters
        self.time = 0.0
        self.radius = 5.0
        self.velocity_z = -1.0  # Upward velocity
        
        self.get_logger().info('Drone velocity control node started')
    
    def publish_offboard_control_mode(self):
        """Set control mode to velocity"""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_velocity_setpoint(self, vx, vy, vz, yaw_rate=0.0):
        """Publish velocity setpoint"""
        msg = TrajectorySetpoint()
        msg.velocity = [vx, vy, vz]
        msg.yawspeed = yaw_rate
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def timer_callback(self):
        """Generate circular trajectory with constant ascent"""
        self.time += 0.05
        
        # Circular motion in xy-plane
        vx = -math.sin(self.time) * self.radius
        vy = math.cos(self.time) * self.radius
        
        # Constant upward velocity
        vz = self.velocity_z
        
        self.publish_offboard_control_mode()
        self.publish_velocity_setpoint(vx, vy, vz, 0.2)
        
        # Log every 20 cycles
        if int(self.time * 20) % 20 == 0:
            self.get_logger().info(
                f"Velocity: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = DroneVelocityControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

