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
from px4_msgs.msg import ActuatorServos
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
        self.actuator_servo_publisher = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', qos_profile)
    
        # Create subscribers
        self.vehicle_local_position_subscriber = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_subscriber = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, qos_profile)
        self.vehicle_attitude_subscriber = self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, qos_profile)

        # Initialize variables
        self.vehicle_local_position = [0, 0, 0]
        self.vehicle_attitude = [0, 0, 0]
        self.fly_mode = VehicleStatus().NAVIGATION_STATE_MAX
        
        self.wp = [[100, 100, -50],
                   [-100, 100, -60],
                   [-100, -100, -50],
                   [100, -100, -60]]
        self.wpIdx = 0
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

    def set_loiter(self):
        """Switch to loitering."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=4.0, param3=3.0)

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

    def publish_position_setpoint(self, wp):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [float(wp[0]), float(wp[1]), float(wp[2])]
        msg.velocity = [np.nan, np.nan, np.nan]
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_publisher.publish(msg)

    def publish_attitude_setpoint(self, att, thrust):
        """Publish the attitude setpoint."""
        msg = VehicleAttitudeSetpoint()
        msg.roll_body = att[0] * math.pi/180
        msg.pitch_body = att[1] * math.pi/180
        # msg.yaw_body = att[2] * math.pi/180
        msg.thrust_body = [float(thrust), 0.0, 0.0]
        self.attitude_setpoint_publisher.publish(msg)

    def publish_actuactor_setpoint(self, thrust, servos):
        """Publish the motors/servos setpoint."""
        msg_motor = ActuatorMotors()
        msg_servo = ActuatorServos()
        msg_motor.control[0] = float(thrust)
        msg_servo.control[0] = float(servos[0])   # left aileron
        msg_servo.control[1] = -float(servos[0])  # right aileron
        msg_servo.control[2] = float(servos[1])   # elevator
        msg_servo.control[3] = float(servos[2])   # rudder
        self.actuator_motor_publisher.publish(msg_motor)
        self.actuator_servo_publisher.publish(msg_servo)


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
            diff = [(ai - bi) ** 2 for ai, bi in zip(self.wp[self.wpIdx], self.vehicle_local_position)]
            distance_error = abs(math.sqrt(sum(diff)))
            self.publish_position_setpoint(self.wp[self.wpIdx])
            if distance_error < 3:
                self.wpIdx += 1
                if self.wpIdx >= len(self.wp):
                    self.set_loiter()
                    self.wpIdx -= 1
            if sim_time%2 == 0:
                print(sim_time, "\twaypoint : ", self.wpIdx+1, "/", len(self.wp), "\t3D error : ", round(distance_error,2))

        if self.Mode == 'att' and self.fly_mode == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            thrust = 0.6
            if sim_time > 5 and sim_time < 10:
                desired_attitude = [-30, 0, 0]  # Roll, Pitch
                self.publish_attitude_setpoint(desired_attitude, thrust)
            elif sim_time > 15 and sim_time < 20:
                desired_attitude = [30, 0, 0]  # Roll, Pitch
                self.publish_attitude_setpoint(desired_attitude, thrust)
            elif sim_time > 25 and sim_time < 30:
                desired_attitude = [0, -30, 0]  # Roll, Pitch
                self.publish_attitude_setpoint(desired_attitude, thrust)
            elif sim_time > 35 and sim_time < 40:
                desired_attitude = [0, 30, 0]  # Roll, Pitch
                self.publish_attitude_setpoint(desired_attitude, thrust)
            else:
                desired_attitude = [0, 0, 0]  # Roll, Pitch
                self.publish_attitude_setpoint(desired_attitude, thrust)
            if sim_time > 45:
                self.set_loiter()
            if sim_time%2 == 0:
                print(sim_time, "\tCurrent", np.round(self.vehicle_attitude,2), "\tTarget", desired_attitude)

        if self.Mode == 'act' and self.fly_mode == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            motor = 0.2
            if sim_time>3 and sim_time<6:
                control_surface = [0.2, 0, 0]
                self.publish_actuactor_setpoint(motor, control_surface)
            elif sim_time>9 and sim_time<12:
                control_surface = [-0.2, 0, 0]
                self.publish_actuactor_setpoint(motor, control_surface)
            elif sim_time>15 and sim_time<18:
                control_surface = [0, 0.2, 0]
                self.publish_actuactor_setpoint(motor, control_surface)
            elif sim_time>21 and sim_time<24:
                control_surface = [0, -0.2, 0]
                self.publish_actuactor_setpoint(motor, control_surface)
            else:
                control_surface = [0, 0, 0]  # aileron, elevator, rudder
                self.publish_actuactor_setpoint(motor, control_surface)
            if sim_time > 27:
                self.set_loiter()
            if sim_time%2 == 0:
                print(sim_time, "\tcontrol surface", control_surface)

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