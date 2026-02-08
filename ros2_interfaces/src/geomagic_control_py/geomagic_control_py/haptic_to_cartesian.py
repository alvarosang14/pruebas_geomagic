from multiprocessing.util import get_logger
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ABBRobotEGM import EGM
import PyKDL

def from_msg_pose(msg_pose, kdl_frame):
    """Convierte geometry_msgs/Pose a KDL.Frame"""
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


def from_msg_pose_stamped(msg_pose_stamped, kdl_frame):
    """Convierte geometry_msgs/PoseStamped a KDL.Frame"""
    kdl_frame.p = PyKDL.Vector(
        msg_pose_stamped.pose.position.x,
        msg_pose_stamped.pose.position.y,
        msg_pose_stamped.pose.position.z
    )
    kdl_frame.M = PyKDL.Rotation.Quaternion(
        msg_pose_stamped.pose.orientation.x,
        msg_pose_stamped.pose.orientation.y,
        msg_pose_stamped.pose.orientation.z,
        msg_pose_stamped.pose.orientation.w
    )


def to_msg(kdl_frame, msg_pose):
    """Convierte KDL.Frame a geometry_msgs/Pose"""
    msg_pose.position.x = kdl_frame.p.x()
    msg_pose.position.y = kdl_frame.p.y()
    msg_pose.position.z = kdl_frame.p.z()
    
    # GetQuaternion devuelve los valores
    qx, qy, qz, qw = kdl_frame.M.GetQuaternion()
    msg_pose.orientation.x = qx
    msg_pose.orientation.y = qy
    msg_pose.orientation.z = qz
    msg_pose.orientation.w = qw

NODE = 'haptic_to_cartesian'
class HapticToCartesian(Node):

    def __init__(self):
        super().__init__(NODE)

        self.first_haptic_output = False
        self.first_teo_output = False
        self.H_0_N_sensor_initial = PyKDL.Frame()
        self.H_0_N_robot_initial = PyKDL.Frame()
        self.H_N_robot_0_sensor = PyKDL.Frame()

        self.abb_manager = None
        self.running = True

        self.set_parameters()
        self.abb_manager_create()
        self.suscribe_create()

        self.get_logger().info(
            "Haptic to Cartesian Node started - Listening to: %s" % (NODE + '/state/pose')
        )

    # -------------------------------------------------------------------------------------------
    def abb_manager_create(self):
        self.abb_manager = EGM()

        while self.running:
            success, state = self.abb_manager.receive_from_robot()
            if not success: 
                continue
            self.cartesian_initial_pose(state)
            self.running = False

        self.get_logger().info('ABB EGM Manager created')
    
    def set_parameters(self):
        descriptor_msg = ParameterDescriptor()
        
        # roll
        roll_N_sensor_ = 0.0
        descriptor_msg.name = "roll_N_sensor"
        descriptor_msg.description = "Rotation about global axis X (roll) from TCP to sensor (radians)."
        descriptor_msg.read_only = True
        descriptor_msg.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("roll_N_sensor", 0.0, descriptor_msg)
        roll_N_sensor_ = self.get_parameter("roll_N_sensor").value
        
        # pitch
        pitch_N_sensor_ = 0.0
        descriptor_msg.name = "pitch_N_sensor"
        descriptor_msg.description = "Rotation about global axis Y (pitch) from TCP to sensor (radians)."
        descriptor_msg.read_only = True
        descriptor_msg.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("pitch_N_sensor", 0.0, descriptor_msg)
        pitch_N_sensor_ = self.get_parameter("pitch_N_sensor").value
        
        # yaw
        yaw_N_sensor_ = 0.0
        descriptor_msg.name = "yaw_N_sensor"
        descriptor_msg.description = "Rotation about global axis Z (yaw) from TCP to sensor (radians)."
        descriptor_msg.read_only = True
        descriptor_msg.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter("yaw_N_sensor", 0.0, descriptor_msg)
        yaw_N_sensor_ = self.get_parameter("yaw_N_sensor").value
        
        self.H_N_robot_0_sensor.M = PyKDL.Rotation.RPY(roll_N_sensor_, pitch_N_sensor_, yaw_N_sensor_)
    
    def suscribe_create(self):
        self.m_haptic_posePhantom_sub = self.create_subscription(
            Pose,
            NODE + '/state/pose',
            self.haptic_pose_callback,
            10
        )
        self.get_logger().info('Haptic subscriber created')

    # -------------------------------------------------------------------------------------------
    def haptic_pose_callback(self, msg):        
        self.haptic_initial_pose(msg)
    
        self.get_logger().info(
            'Received haptic pose: [%.3f, %.3f, %.3f]',
            msg.position.x, msg.position.y, msg.position.z)
        
        self.comprobate_initialPose(msg)

    def comprobate_initialPose(self, msg):
        if (not self.first_haptic_output) and (not self.first_teo_output):
            self.get_logger().warn(
                'Not received initial pose of Haptic %d and Teo %d', 
                self.first_haptic_output, self.first_teo_output
            )
            return
        
        cartesian_cmd = self.calculate_diferential_pose(msg)
        self.run_egm_loop(cartesian_cmd)

        self.get_logger().info(
            'Published cartesian command: [%.3f, %.3f, %.3f]',
            cartesian_cmd.position.x,
            cartesian_cmd.position.y,
            cartesian_cmd.position.z
        )
    
    def calculate_diferential_pose(self, msg):
        H_0_N_sensor_current = PyKDL.Frame()
        from_msg_pose(msg, H_0_N_sensor_current)

        H_sensor_initial_current = self.H_0_N_sensor_initial.Inverse() * H_0_N_sensor_current

        # Solo translacion
        H_sensor_initial_current.M = PyKDL.Rotation.Identity()

        # Solo rotacion
        Rot_0_N_sensor_initial = self.H_0_N_sensor_initial
        Rot_0_N_sensor_initial.p = PyKDL.Vector.Zero()
        # H_N_robot_0_sensor ya nos hemos asegurado que es solo rotacion

        p = self.H_0_N_robot_initial * (self.H_N_robot_0_sensor * (Rot_0_N_sensor_initial * H_sensor_initial_current.p))

        # transformamos la traslacion con respecto la base del sensor
        # tranformamos el resultado a los mismos ejes de teo
        # transformamos el resultado con la base de teo
        # KDL::Frame H_0_N_robot = H_0_N_robot_initial * H_N_robot_0_sensor * Rot_0_N_sensor_initial * H_sensor_initial_current;
        H_0_N_robot = self.H_0_N_robot_initial
        H_0_N_robot.p = p

        out = Pose()
        to_msg(H_0_N_robot, out)
        return out

    # -------------------------------------------------------------------------------------------
    def haptic_initial_pose(self, msg):
        if not self.first_haptic_output:
            self.first_haptic_output = True
            from_msg_pose(msg, self.H_0_N_sensor_initial)
            self.get_logger().info('Initial haptic pose correctly set.')
    
    def cartesian_initial_pose(self, state):
        if not self.first_teo_output:
            self.first_teo_output = True
            self.H_0_N_robot_initial.p = [state.cartesian.pos.x, state.cartesian.pos.y, state.cartesian.pos.z]
            self.H_0_N_robot_initial.M = [state.cartesian.orient.u0, state.cartesian.orient.u1, state.cartesian.orient.u2, state.cartesian.orient.u3]
            self.get_logger().info('Initial cartesian pose correctly set.')

    # -------------------------------------------------------------------------------------------
    def run_egm_loop(self, cartesian_cmd):
        # Interpolación de Posición
        self.current_send_pos[0] = self.interpolate(self.current_send_pos[0], self.target_pos[0], SMOOTH_FACTOR)
        self.current_send_pos[1] = self.interpolate(self.current_send_pos[1], self.target_pos[1], SMOOTH_FACTOR)
        self.current_send_pos[2] = self.interpolate(self.current_send_pos[2], self.target_pos[2], SMOOTH_FACTOR)

        self.abb_manager.send_to_robot(cartesian=(self.current_send_pos, self.current_send_orient))
    
def main(args=None):
    try:
        with rclpy.init(args=args):
            rclpy.spin(HapticToCartesian())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()