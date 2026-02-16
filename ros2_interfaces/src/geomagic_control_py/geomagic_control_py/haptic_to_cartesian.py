import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Pose
from std_msgs.msg import Int32MultiArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ABBRobotEGM import EGM
import PyKDL

# MODULE MainModule
#     VAR egmident egmID1;
#     VAR egmstate egmSt1;

#     CONST jointtarget initial:=[[0,0,0,0,30,0],[9E09,9E09,9E09,9E09,9E09,9E09]];
#     CONST egm_minmax egm_minmax_lin:=[-1E-9,+1E-9]; ! [mm]
#     CONST egm_minmax egm_minmax_rot:=[-1E-9,+1E-9]; ! [deg]

#     VAR pose corr_frame_offs:=[[0,0,0],[1,0,0,0]];
    
#     PROC main()
#         MoveAbsJ initial,v500,fine,tool0;
        
#         EGMReset egmID1;
#         EGMGetId egmID1;
#         egmSt1:=EGMGetState(egmID1);
        
#         IF egmSt1<=EGM_STATE_CONNECTED THEN
#             EGMSetupUC ROB_1,egmID1,"default","UCdevice"\pose;
#         ENDIF

#         EGMActPose egmID1\StreamStart\Tool:=tool0\DIFromSensor:=diEGM,corr_frame_offs,EGM_FRAME_WOBJ,corr_frame_offs,EGM_FRAME_WOBJ
#             \X:=egm_minmax_lin\Y:=egm_minmax_lin\Z:=egm_minmax_lin
#             \RX:=egm_minmax_rot\RY:=egm_minmax_rot\RZ:=egm_minmax_rot
#             \MaxSpeedDeviation:=4000;
#         EGMRunPose egmID1,EGM_STOP_HOLD\NoWaitCond\x\y\z\rx\ry\rz\RampInTime:=0.05;
#         WaitDI diEGM,1;
#         EGMStop egmID1,EGM_STOP_HOLD;
#     ENDPROC
# ENDMODULE


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


NODE = 'haptic_to_cartesian'
HAPTIC_NODE_NAME = 'haptic_device_ros2'

class HapticToCartesian(Node):

    def __init__(self):
        super().__init__(NODE)

        self.first_haptic_output = False
        self.first_teo_output = False
        self.capture_enabled = True
        self.H_0_N_sensor_initial = PyKDL.Frame()
        self.H_0_N_robot_initial = PyKDL.Frame()
        self.H_N_robot_0_sensor = PyKDL.Frame()

        self.abb_manager = None

        self.set_parameters()
        self.abb_manager_create()
        self.suscribe_create()

        self.get_logger().info(
            f'Haptic to Cartesian Node started - Listening to: {NODE}/state/pose'
        )

    # -------------------------------------------------------------------------------------------
    def abb_manager_create(self):
        self.abb_manager = EGM()

        count = 0
        while count < 10:
            success, state = self.abb_manager.receive_from_robot()
            if not success:
                count += 1
                continue
            self.cartesian_initial_pose(state)
            break

        if count == 10:
            self.get_logger().error('Failed to receive initial state from ABB robot after 10 attempts.')
            raise RuntimeError('Failed to receive initial state from ABB robot after 10 attempts.')

        self.get_logger().info(f'Initial robot pose: {state.cartesian}')
        self.get_logger().info('ABB EGM Manager created')

    def set_parameters(self):
        descriptor_msg = ParameterDescriptor()
        
        # roll
        roll_N_sensor_ = 0.0
        descriptor_msg.name = 'roll_N_sensor'
        descriptor_msg.description = 'Rotation about global axis X (roll) from TCP to sensor (radians).'
        descriptor_msg.read_only = True
        descriptor_msg.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter('roll_N_sensor', 0.0, descriptor_msg)
        roll_N_sensor_ = self.get_parameter('roll_N_sensor').value
        
        # pitch
        pitch_N_sensor_ = 0.0
        descriptor_msg.name = 'pitch_N_sensor'
        descriptor_msg.description = 'Rotation about global axis Y (pitch) from TCP to sensor (radians).'
        descriptor_msg.read_only = True
        descriptor_msg.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter('pitch_N_sensor', 0.0, descriptor_msg)
        pitch_N_sensor_ = self.get_parameter('pitch_N_sensor').value
        
        # yaw
        yaw_N_sensor_ = 0.0
        descriptor_msg.name = 'yaw_N_sensor'
        descriptor_msg.description = 'Rotation about global axis Z (yaw) from TCP to sensor (radians).'
        descriptor_msg.read_only = True
        descriptor_msg.type = ParameterType.PARAMETER_DOUBLE
        self.declare_parameter('yaw_N_sensor', 0.0, descriptor_msg)
        yaw_N_sensor_ = self.get_parameter('yaw_N_sensor').value
        
        self.H_N_robot_0_sensor.M = PyKDL.Rotation.RPY(roll_N_sensor_, pitch_N_sensor_, yaw_N_sensor_)
    
    def suscribe_create(self):
        self.m_haptic_poseHaptic_sub = self.create_subscription(
            Pose,
            HAPTIC_NODE_NAME + '/state/pose',
            self.haptic_pose_callback,
            10
        )

        self.m_haptic_buttons_sub = self.create_subscription(
            Int32MultiArray,
            HAPTIC_NODE_NAME + '/state/buttons',
            self.haptic_buttons_callback,
            10
        )
        self.get_logger().info('Haptic subscriber created')

    # -------------------------------------------------------------------------------------------
    def haptic_buttons_callback(self, msg):
        if len(msg.data) < 2:
            return
        
        # boton 0: reset de posiciones iniciales
        if msg.data[0] == 1:
            self.first_teo_output = False
            self.get_logger().info('Button 0 pressed: Initial positions reset. Ready to recalibrate.')
        
        # boton 1: enable/disable captura haptica
        if msg.data[1] == 1:
            self.capture_enabled = not self.capture_enabled
            self.get_logger().info(f'Button 1 pressed: Haptic capture {self.capture_enabled}')

    def haptic_pose_callback(self, msg):
        if not self.capture_enabled:
            self.get_logger().info('Haptic capture disabled, ignoring pose message.')
            return

        self.haptic_initial_pose(msg)

        self.get_logger().info(
            f'Received haptic pose: [{msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}]'
        )

        self.comprobate_initialPose(msg)

    def comprobate_initialPose(self, msg):
        if (not self.first_haptic_output) or (not self.first_teo_output):
            self.get_logger().warn(
                f'Not received initial pose of Haptic {self.first_haptic_output} and Teo {self.first_teo_output}'
            )
            return

        cartesian_cmd = self.calculate_diferential_pose(msg)
        self.run_egm_loop(cartesian_cmd)

    def calculate_diferential_pose(self, msg):
        H_0_N_sensor_current = PyKDL.Frame()
        from_msg_pose(msg, H_0_N_sensor_current)

        H_sensor_initial_current = self.H_0_N_sensor_initial.Inverse() * H_0_N_sensor_current

        # Solo translacion
        H_sensor_initial_current.M = PyKDL.Rotation.Identity()

        # Solo rotacion
        Rot_0_N_sensor_initial = PyKDL.Frame(self.H_0_N_sensor_initial)
        Rot_0_N_sensor_initial.p = PyKDL.Vector.Zero()
        # H_N_robot_0_sensor ya nos hemos asegurado que es solo rotacion

        p = self.H_0_N_robot_initial * (self.H_N_robot_0_sensor * (Rot_0_N_sensor_initial * H_sensor_initial_current.p))

        # transformamos la traslacion con respecto la base del sensor
        # tranformamos el resultado a los mismos ejes de teo
        # transformamos el resultado con la base de teo
        # KDL::Frame H_0_N_robot = H_0_N_robot_initial * H_N_robot_0_sensor * Rot_0_N_sensor_initial * H_sensor_initial_current;
        H_0_N_robot = PyKDL.Frame(self.H_0_N_robot_initial)
        H_0_N_robot.p = p

        return H_0_N_robot

    # -------------------------------------------------------------------------------------------
    def haptic_initial_pose(self, msg):
        if not self.first_haptic_output:
            self.first_haptic_output = True
            from_msg_pose(msg, self.H_0_N_sensor_initial)
            self.get_logger().info('Initial haptic pose correctly set.')
    
    def cartesian_initial_pose(self, state):
        if not self.first_teo_output:
            self.first_teo_output = True
            from_state_abb(state, self.H_0_N_robot_initial)
            self.get_logger().info('Initial cartesian pose correctly set.')

    # -------------------------------------------------------------------------------------------
    def run_egm_loop(self, cartesian_cmd):
        pos_mm, orient = to_state_abb(cartesian_cmd)
        self.abb_manager.send_to_robot(cartesian=(pos_mm, orient),digital_signal_to_robot=False)
        self.get_logger().info(
            f'Published cartesian command: [{pos_mm[0]:.3f}, {pos_mm[1]:.3f}, {pos_mm[2]:.3f}]'
        )
    
def main(args=None):
    rclpy.init(args=args)
    node = HapticToCartesian()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()