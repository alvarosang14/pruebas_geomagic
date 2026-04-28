import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Wrench, Pose
from std_msgs.msg import Int32MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ABBRobotEGM import EGM
import PyKDL


NODE = 'haptic_cartesian_bridge'
HAPTIC_NODE_NAME = 'haptic_device_ros2'


# ===================== AUX FUNCTIONS =====================
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


# ===================== NODE =====================
class HapticCartesianBridge(Node):

    def __init__(self):
        super().__init__(NODE)

        # -------- STATE --------
        self.first_haptic_output = False
        self.first_robot_output = False
        self.capture_enabled = True

        self.H_0_N_sensor_initial = PyKDL.Frame()
        self.H_0_N_robot_initial = PyKDL.Frame()
        self.H_N_robot_0_sensor = PyKDL.Frame()

        self.scale_ = 1.0

        # -------- INIT --------
        self.set_parameters()
        self.create_egm()
        self.create_haptic_interface()
        self.create_subscribers()

        self.get_logger().info('Unified Haptic-Cartesian Bridge started')

    # ===================== HAPTIC =====================
    def create_haptic_interface(self):
        # Publisher feedback
        self.feedback_pub = self.create_publisher(
            JointState,
            HAPTIC_NODE_NAME + '/feedback',
            10
        )

        # Set feedback mode
        self.client = self.create_client(
            SetBool,
            HAPTIC_NODE_NAME + '/set_feedback_mode'
        )

        req = SetBool.Request()
        req.data = False

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for haptic service...')

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            raise RuntimeError("Failed to set feedback mode")

    # ===================== EGM =====================
    def create_egm(self):
        self.abb = EGM()

        for _ in range(10):
            success, state = self.abb.receive_from_robot()
            if success:
                self.cartesian_initial_pose(state)
                break
        else:
            raise RuntimeError("No EGM connection")

        self.get_logger().info("EGM initialized")

    # ===================== PARAMETERS =====================
    def set_parameters(self):
        self.declare_parameter('scale', 1.0)
        self.scale_ = self.get_parameter('scale').value

        self.declare_parameter('roll_N_sensor', 0.0)
        self.declare_parameter('pitch_N_sensor', 0.0)
        self.declare_parameter('yaw_N_sensor', 0.0)

        r = self.get_parameter('roll_N_sensor').value
        p = self.get_parameter('pitch_N_sensor').value
        y = self.get_parameter('yaw_N_sensor').value

        self.H_N_robot_0_sensor.M = PyKDL.Rotation.RPY(r, p, y)

    # ===================== SUBSCRIBERS =====================
    def create_subscribers(self):
        self.create_subscription(Wrench, '/state', self.force_callback, 10)
        self.create_subscription(Pose, HAPTIC_NODE_NAME + '/state/pose', self.pose_callback, 10)
        self.create_subscription(Int32MultiArray, HAPTIC_NODE_NAME + '/state/buttons', self.buttons_callback, 10)

    # ===================== CALLBACKS =====================
    def force_callback(self, msg):
        self.feedback_pub.publish(msg)

    def buttons_callback(self, msg):
        if len(msg.data) < 2:
            return

        if msg.data[0] == 1:
            self.first_robot_output = False

        if msg.data[1] == 1:
            self.capture_enabled = not self.capture_enabled

    def pose_callback(self, msg):
        if not self.capture_enabled:
            return

        self.set_initial_haptic(msg)

        if not (self.first_haptic_output and self.first_robot_output):
            return

        cmd = self.compute_motion(msg)
        self.send_to_robot(cmd)

    # ===================== LOGIC =====================
    def set_initial_haptic(self, msg):
        if not self.first_haptic_output:
            from_msg_pose(msg, self.H_0_N_sensor_initial)
            self.first_haptic_output = True

    def cartesian_initial_pose(self, state):
        if not self.first_robot_output:
            from_state_abb(state, self.H_0_N_robot_initial)
            self.first_robot_output = True

    def compute_motion(self, msg):
        current = PyKDL.Frame()
        from_msg_pose(msg, current)

        delta = self.H_0_N_sensor_initial.Inverse() * current
        delta.M = PyKDL.Rotation.Identity()

        p = self.H_0_N_robot_initial * (
            self.H_N_robot_0_sensor * delta.p
        )

        p = PyKDL.Vector(
            p.x() * self.scale_,
            p.y() * self.scale_,
            p.z() * self.scale_
        )

        result = PyKDL.Frame(self.H_0_N_robot_initial)
        result.p = p

        return result

    def send_to_robot(self, frame):
        pos, orient = to_state_abb(frame)
        self.abb.send_to_robot(cartesian=(pos, orient), digital_signal_to_robot=False)


# ===================== MAIN =====================
def main(args=None):
    rclpy.init(args=args)
    node = HapticCartesianBridge()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()