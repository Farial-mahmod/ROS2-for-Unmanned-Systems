#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
from px4_msgs.msg import VehicleStatus
import math

class DroneOffboardControl(Node):
    """Simple drone offboard control example"""
    
    def __init__(self):
        super().__init__('drone_offboard_control')
        
        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        
        # Subscriber for vehicle status
        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', 
            self.vehicle_status_callback, 10)
        
        # State variables
        self.offboard_setpoint_counter = 0
        self.vehicle_armed = False
        self.vehicle_nav_state = 0
        self.takeoff_height = -5.0  # NED coordinates: negative = up
        
        # Control timer (100ms)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Drone offboard control node started')
    
    def vehicle_status_callback(self, msg):
        """Update vehicle status"""
        self.vehicle_armed = msg.arming_state == 2  # 2 = armed
        self.vehicle_nav_state = msg.nav_state
    
    def publish_offboard_control_mode(self):
        """Publish offboard control mode"""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_trajectory_setpoint(self):
        """Publish trajectory setpoint for hover at takeoff height"""
        msg = TrajectorySetpoint()
        msg.position = [0.0, 0.0, self.takeoff_height]
        msg.yaw = -3.14  # -180 degrees
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
    
    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """Publish vehicle command"""
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
    
    def arm(self):
        """Arm the drone"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info('Arm command sent')
    
    def timer_callback(self):
        """Main control loop"""
        if self.offboard_setpoint_counter == 10:
            # Change to offboard mode after 10 setpoints
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1, 6)
            self.arm()
        
        # Required: publish offboard mode and trajectory setpoint
        self.publish_offboard_control_mode()
        self.publish_trajectory_setpoint()
        
        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1
        
        # Log status periodically
        if self.offboard_setpoint_counter % 20 == 0:
            self.get_logger().info(
                f"State - Armed: {self.vehicle_armed}, "
                f"Nav State: {self.vehicle_nav_state}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = DroneOffboardControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


This example sends velocity commands to the drone:

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

