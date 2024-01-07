#!/usr/bin/env python
############################################################################
#
#   Copyright (C) 2022 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
import math
import time
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleStatus
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleAttitudeSetpoint
from px4_msgs.msg import ActuatorMotors
from px4_msgs.msg import VehicleAttitude
from navpy import angle2quat
from navpy import quat2angle

class OffboardControl(Node):
    """Node for controlling a vehicle in offboard mode."""

    def __init__(self) -> None:
        super().__init__('offboard')

        # Configure QoS profile for publishing and subscribing
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Declare and get a parameter from command-line arguments
        self.declare_parameter('Mode', 'pos')
        self.Mode = self.get_parameter('Mode').get_parameter_value().string_value
        self.get_logger().info(f'Mode = {self.Mode}')

        # Create publishers
        self.offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.attitude_setpoint_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', qos_profile)
        self.actuator_motor_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile)
    
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = [0, 0, 0]
        self.vehicle_attitude = [0, 0, 0]
        self.fly_mode = VehicleStatus().NAVIGATION_STATE_MAX
        self.start_time = time.time()

        # Create a timer to publish control commands
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10Hz

    def vehicle_local_position_callback(self, msg):
        """Callback function for vehicle_local_position topic subscriber."""
        self.vehicle_local_position = [msg.x, msg.y, msg.z]

    def vehicle_status_callback(self, msg):
        """Callback function for vehicle_status topic subscriber."""
        self.fly_mode = msg.nav_state

    def vehicle_attitude_callback(self, msg):
        """Callback function for vehicle_attitude topic subscriber."""
        q0 = msg.q[0]
        qvec = [msg.q[1], msg.q[2], msg.q[3]]
        psi, theta, phi = quat2angle(q0, qvec, output_unit='deg', rotation_sequence='ZYX')
        self.vehicle_attitude = [phi, theta, psi]

    def arm(self):
        """Send an arm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.get_logger().info('Arm command sent')

    def disarm(self):
        """Send a disarm command to the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def change_flight_mode(self, val):
        """Switch to flight mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=float(val))
        if val == 1:
            self.get_logger().info("Switching to manual mode")
        elif val == 2:
            self.get_logger().info("Switching to altitude mode")
        if val == 3:
            self.get_logger().info("Switching to position mode")
        elif val == 4:
            self.get_logger().info("Switching to auto mode")
        elif val == 5:
            self.get_logger().info("Switching to acro mode")
        elif val == 6:
            self.get_logger().info("Switching to offboard mode")
        elif val == 7:
            self.get_logger().info("Switching to stabilized mode")

    def land(self):
        """Switch to land mode."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info("Switching to land mode")

    def publish_offboard_control_heartbeat_signal(self, Mode):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.direct_actuator = False
        if Mode == 'pos':
            msg.position = True
        elif Mode == 'att':
            msg.attitude = True
        elif Mode == 'act':
            msg.direct_actuator = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_publisher.publish(msg)

    def publish_position_setpoint(self, wp, yaw):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(wp[0]), float(wp[1]), float(wp[2])]
        msg.yaw = yaw * math.pi/180
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_attitude_setpoint(self, att, thrust):
        """Publish the attitude setpoint."""
        q0, qvec = angle2quat(float(att[2]), float(att[1]), float(att[0]), input_unit='deg', rotation_sequence='ZYX')
        msg = VehicleAttitudeSetpoint()
        msg.q_d = [q0, qvec[0], qvec[1], qvec[2]]
        msg.thrust_body = [0.0, 0.0, -float(thrust)]
        self.attitude_setpoint_publisher.publish(msg)

    def publish_motors_setpoint(self, motors):
        """Publish the motors setpoint."""
        msg = ActuatorMotors()
        msg.control[0] = float(motors[0])
        msg.control[1] = float(motors[1])
        msg.control[2] = float(motors[2])
        msg.control[3] = float(motors[3])
        self.actuator_motor_publisher.publish(msg)

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command."""
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_publisher.publish(msg)

    def timer_callback(self) -> None:
        """Callback function for the timer."""
        sim_time = round(time.time() - self.start_time, 2) 
        self.publish_offboard_control_heartbeat_signal(self.Mode)
        if sim_time == 1.0:
            self.change_flight_mode(6)

        if self.Mode == 'pos' and self.fly_mode == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            waypoint = [-50, 50, -15]
            target_heading = 45
            self.publish_position_setpoint(waypoint, target_heading)
            diff = [(ai - bi) ** 2 for ai, bi in zip(waypoint, self.vehicle_local_position)]
            distance_error = abs(math.sqrt(sum(diff)))
            hor_error = math.sqrt((waypoint[0]-self.vehicle_local_position[0])**2 + (waypoint[1]-self.vehicle_local_position[1])**2)
            if distance_error < 3:
                self.change_flight_mode(3)
            if sim_time%2 == 0:
                print(sim_time, "\t2D error : ", round(hor_error,2), "\t3D error : ", round(distance_error,2))

        if self.Mode == 'att' and self.fly_mode == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            thrust = 0.8
            if sim_time > 5 and sim_time < 10:
                desired_attitude = [-10, 0, 0]  # Roll, Pitch, Yaw
                self.publish_attitude_setpoint(desired_attitude, thrust)
            elif sim_time > 15 and sim_time < 20:
                desired_attitude = [10, 0, 0]  # Roll, Pitch, Yaw
                self.publish_attitude_setpoint(desired_attitude, thrust)
            elif sim_time > 25 and sim_time < 30:
                desired_attitude = [0, -10, 0]  # Roll, Pitch, Yaw
                self.publish_attitude_setpoint(desired_attitude, thrust)
            elif sim_time > 35 and sim_time < 40:
                desired_attitude = [0, 10, 0]  # Roll, Pitch, Yaw
                self.publish_attitude_setpoint(desired_attitude, thrust)
            else:
                desired_attitude = [0, 0, 0]  # Roll, Pitch, Yaw
                self.publish_attitude_setpoint(desired_attitude, thrust)
            if sim_time > 45:
                self.change_flight_mode(3)
            if sim_time%2 == 0:
                print(sim_time, "\tCurrent", np.round(self.vehicle_attitude,2), "\tTarget", desired_attitude)

        if self.Mode == 'act' and self.fly_mode == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            motors = [1, 1, 1, 1]  # 0~1
            self.publish_motors_setpoint(motors)


def main(args=None) -> None:
    print('Starting offboard control node...')
    rclpy.init(args=args)
    offboard_control = OffboardControl()
    rclpy.spin(offboard_control)
    offboard_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)