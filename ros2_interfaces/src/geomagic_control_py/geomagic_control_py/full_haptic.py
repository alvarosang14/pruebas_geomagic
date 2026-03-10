import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ABBRobotEGM import EGM
import PyKDL


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def from_msg_pose(msg_pose, kdl_frame):
    kdl_frame.p = PyKDL.Vector(
        msg_pose.position.x,
        msg_pose.position.y,
        msg_pose.position.z
    )
    kdl_frame.M = PyKDL.Rotation.Quaternion(
        msg_pose.orientation.x,
        msg_pose.orientation.y,
        msg_pose.orientation.z,
        msg_pose.orientation.w
    )


def from_state_abb(state, kdl_frame):
    kdl_frame.p = PyKDL.Vector(
        state.cartesian.pos.x * 0.001,
        state.cartesian.pos.y * 0.001,
        state.cartesian.pos.z * 0.001
    )
    kdl_frame.M = PyKDL.Rotation.Quaternion(
        state.cartesian.orient.u1,
        state.cartesian.orient.u2,
        state.cartesian.orient.u3,
        state.cartesian.orient.u0
    )


def to_state_abb(kdl_frame):
    pos_mm = [
        kdl_frame.p.x() * 1000.0,
        kdl_frame.p.y() * 1000.0,
        kdl_frame.p.z() * 1000.0
    ]
    qx, qy, qz, qw = kdl_frame.M.GetQuaternion()
    orient = [qw, qx, qy, qz]
    return pos_mm, orient


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

NODE = 'haptic_egm_node'
HAPTIC_NODE_NAME = 'haptic_device_ros2'


class HapticEGMNode(Node):

    def __init__(self):
        super().__init__(NODE)

        # Haptic → Cartesian
        self.first_haptic_output = False
        self.first_robot_output = False
        self.capture_enabled = True
        self.H_0_N_sensor_initial = PyKDL.Frame()
        self.H_0_N_robot_initial = PyKDL.Frame()
        self.H_N_robot_0_sensor = PyKDL.Frame()

        # Cartesian → Haptic feedback
        self.H_0_N_robot_reference = PyKDL.Frame()
        self.feedback_gain = 1.0

        # Único EGM compartido
        self.abb_manager = None

        self._setup_parameters()
        self._setup_egm()
        self._setup_subscribers()
        self._setup_publishers()

        # Loop feedback 100 Hz
        self.timer = self.create_timer(0.01, self._feedback_loop)

        self.get_logger().info(f'{NODE} started')

    # -----------------------------------------------------------------------
    # Setup
    # -----------------------------------------------------------------------

    def _setup_parameters(self):
        desc = ParameterDescriptor()
        desc.read_only = True
        desc.type = ParameterType.PARAMETER_DOUBLE

        for axis in ('roll', 'pitch', 'yaw'):
            desc.name = f'{axis}_N_sensor'
            desc.description = f'Rotation ({axis}) TCP->sensor (rad).'
            self.declare_parameter(f'{axis}_N_sensor', 0.0, desc)

        roll  = self.get_parameter('roll_N_sensor').value
        pitch = self.get_parameter('pitch_N_sensor').value
        yaw   = self.get_parameter('yaw_N_sensor').value
        self.H_N_robot_0_sensor.M = PyKDL.Rotation.RPY(roll, pitch, yaw)

    def _setup_egm(self):
        self.abb_manager = EGM()

        for _ in range(10):
            success, state = self.abb_manager.receive_from_robot()
            if not success:
                continue

            # Pose inicial para calculo diferencial de movimiento
            from_state_abb(state, self.H_0_N_robot_initial)
            self.first_robot_output = True

            # Pose de referencia para feedback de fuerza
            self.H_0_N_robot_reference.p = PyKDL.Vector(
                state.cartesian.pos.x * 0.001,
                state.cartesian.pos.y * 0.001,
                state.cartesian.pos.z * 0.001
            )

            self.get_logger().info(f'Initial robot pose: {state.cartesian}')
            self.get_logger().info('EGM ready')
            return

        raise RuntimeError('Failed to receive initial state from ABB robot after 10 attempts.')

    def _setup_subscribers(self):
        self.create_subscription(Pose,
            HAPTIC_NODE_NAME + '/state/pose', self._haptic_pose_callback, 10)
        self.create_subscription(Int32MultiArray,
            HAPTIC_NODE_NAME + '/state/buttons', self._haptic_buttons_callback, 10)

    def _setup_publishers(self):
        self.feedback_pub = self.create_publisher(
            JointState, HAPTIC_NODE_NAME + '/feedback', 10)

    # -----------------------------------------------------------------------
    # Haptic -> Cartesian (movimiento del robot)
    # -----------------------------------------------------------------------

    def _haptic_buttons_callback(self, msg):
        if len(msg.data) < 2:
            return
        if msg.data[0] == 1:
            self.first_robot_output = False
            self.get_logger().info('Button 0: reset pose inicial, recalibrando...')
        if msg.data[1] == 1:
            self.capture_enabled = not self.capture_enabled
            self.get_logger().info(
                f'Button 1: captura haptica {"ON" if self.capture_enabled else "OFF"}')

    def _haptic_pose_callback(self, msg):
        if not self.capture_enabled:
            return

        if not self.first_haptic_output:
            self.first_haptic_output = True
            from_msg_pose(msg, self.H_0_N_sensor_initial)
            self.get_logger().info('Initial haptic pose set.')

        if not self.first_robot_output:
            self.get_logger().warn('Esperando pose inicial del robot...')
            return

        cartesian_cmd = self._calculate_differential_pose(msg)
        pos_mm, orient = to_state_abb(cartesian_cmd)
        self.abb_manager.send_to_robot(cartesian=(pos_mm, orient), digital_signal_to_robot=False)
        self.get_logger().debug(
            f'Cartesian cmd: [{pos_mm[0]:.3f}, {pos_mm[1]:.3f}, {pos_mm[2]:.3f}] mm')

    def _calculate_differential_pose(self, msg):
        H_0_N_sensor_current = PyKDL.Frame()
        from_msg_pose(msg, H_0_N_sensor_current)

        H_sensor_initial_current = self.H_0_N_sensor_initial.Inverse() * H_0_N_sensor_current
        H_sensor_initial_current.M = PyKDL.Rotation.Identity()  # solo traslacion

        Rot_0_N_sensor_initial = PyKDL.Frame(self.H_0_N_sensor_initial)
        Rot_0_N_sensor_initial.p = PyKDL.Vector.Zero()

        p = self.H_0_N_robot_initial * (
            self.H_N_robot_0_sensor * (
                Rot_0_N_sensor_initial * H_sensor_initial_current.p
            )
        )

        H_0_N_robot = PyKDL.Frame(self.H_0_N_robot_initial)
        H_0_N_robot.p = p
        return H_0_N_robot

    # -----------------------------------------------------------------------
    # Cartesian -> Haptic (feedback de fuerza)
    # -----------------------------------------------------------------------

    def _feedback_loop(self):
        if not self.first_robot_output:
            return

        success, state = self.abb_manager.receive_from_robot()
        if not success:
            self.get_logger().warn('No se recibio estado del robot', throttle_duration_sec=1.0)
            return

        current_pos = PyKDL.Vector(
            state.cartesian.pos.x * 0.001,
            state.cartesian.pos.y * 0.001,
            state.cartesian.pos.z * 0.001
        )

        diff = self.H_0_N_robot_reference.p - current_pos
        gain = self.feedback_gain * 1000.0

        feedback_msg = JointState()
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.effort = [diff.x() * gain, diff.y() * gain, diff.z() * gain]
        self.feedback_pub.publish(feedback_msg)

        self.get_logger().debug(
            f'Feedback: [{feedback_msg.effort[0]:.3f}, '
            f'{feedback_msg.effort[1]:.3f}, '
            f'{feedback_msg.effort[2]:.3f}] N')


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = HapticEGMNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()