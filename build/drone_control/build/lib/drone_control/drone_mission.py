#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus

class DroneMission(Node):
    def __init__(self):
        super().__init__("drone_mission")
        self.get_logger().info("DroneMission has been started.")

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        
        # Initialize variables
        self.offboard_setpoint_counter = 0
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.setpoint = 1
        self.reached_target_altitude = False
        self.start_time = None

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)

    # Callback function for vehicle_local_position topic subscriber.
    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position

    # Callback function for vehicle_status topic subscriber.
    def vehicle_status_callback(self, vehicle_status):  
        self.vehicle_status = vehicle_status

    # Callback function for the timer.
    def timer_callback(self) -> None:
        self.publish_offboard_control_heartbeat_signal1()

        if self.offboard_setpoint_counter == 10:
            self.engage_offboard_mode()
            self.arm()

        if self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            if not self.reached_target_altitude:
                # Ascend to 30 meters
                if self.vehicle_local_position.z > -30:
                    self.publish_position_setpoint(self.x, self.y, -30.0)
                    self.get_logger().info('publishing setpoints')
                else:
                    self.reached_target_altitude = True
                    self.start_time = self.get_clock().now().nanoseconds
                    self.get_logger().info('Reached target altitude of 30 meters')
            else:
                # Apply downward acceleration after reaching 30 meters
                current_time = self.get_clock().now().nanoseconds
                time_elapsed = (current_time - self.start_time) / 1e9  # Convert nanoseconds to seconds

                # Calculate the desired position based on downward acceleration
                target_z = -30.0 + 0.5 * 12.0* time_elapsed ** 2

                # Publish position setpoint with updated Z-axis position
                self.publish_position_setpoint(self.x, self.y, target_z)

                # Log the current Z position
                self.get_logger().info(f'Current Z position: {self.vehicle_local_position.z}, Target Z position: {target_z}')

                # Check for landing condition
                if target_z >= 0:
                    self.land()
                    exit(0)

        if self.offboard_setpoint_counter < 11:
            self.offboard_setpoint_counter += 1

    # Publish the offboard control mode.
    def publish_offboard_control_heartbeat_signal1(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    # Send an arm command to the vehicle.
    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    # Send a disarm command to the vehicle.
    def disarm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    # Switch to offboard mode.
    def engage_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    # Switch to land mode.
    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    # Publish the trajectory setpoint.
    def publish_position_setpoint(self, x: float, y: float, z: float): 
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 1.57079  # (90 degree)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)
        self.get_logger().info(f"Publishing position setpoints {[x, y, z]}")
    
    def publish_acceleration_setpoint(self, az: float):
        msg = TrajectorySetpoint()
        msg.acceleration = [0.0, 0.0, float(az)]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    # Publish a vehicle command.
    def publish_vehicle_command(self, command, **params) -> None:      
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneMission()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
