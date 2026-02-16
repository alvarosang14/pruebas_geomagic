import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from ABBRobotEGM import EGM
import PyKDL

NODE = 'cartesian_to_haptic_feedback'
HAPTIC_NODE_NAME = 'haptic_device_ros2'

class CartesianToHapticFeedback(Node):

    def __init__(self):
        super().__init__(NODE)

        self.H_0_N_robot_reference = PyKDL.Frame()
        self.first_robot_pose = False
        
        self.abb_manager = None
        self.feedback_gain = 1.0  # Ganancia del feedback
        
        self.abb_manager_create()
        self.publisher_create()

        # Timer para leer posición del robot y publicar feedback
        self.timer = self.create_timer(0.01, self.feedback_loop)  # 100Hz

        self.get_logger().info('Cartesian to Haptic Feedback Node started')

    # -------------------------------------------------------------------------------------------
    def abb_manager_create(self):
        # Crear EGM con puerto específico
        self.abb_manager = EGM(port=6511)
        
        count = 0
        while count < 10:
            success, state = self.abb_manager.receive_from_robot()
            if not success:
                count += 1
                continue
            self.set_robot_reference_pose(state)
            break

        if count == 10:
            self.get_logger().error('Failed to receive initial state from ABB robot after 10 attempts.')
            raise RuntimeError('Failed to receive initial state from ABB robot')

        self.get_logger().info(f'Initial robot reference pose: {state.cartesian}')

    def publisher_create(self):
        # Publisher de feedback al háptico
        self.feedback_pub = self.create_publisher(
            JointState,
            HAPTIC_NODE_NAME + '/feedback',
            10
        )
        self.get_logger().info('Feedback publisher created')

    # -------------------------------------------------------------------------------------------
    def set_robot_reference_pose(self, state):
        if not self.first_robot_pose:
            self.first_robot_pose = True
            self.H_0_N_robot_reference.p = PyKDL.Vector(
                state.cartesian.pos.x * 0.001,
                state.cartesian.pos.y * 0.001,
                state.cartesian.pos.z * 0.001
            )
            self.get_logger().info('Robot reference pose set.')

    def feedback_loop(self):
        if not self.first_robot_pose:
            return

        # robot pose
        success, state = self.abb_manager.receive_from_robot()
        if not success:
            self.get_logger().warn('Failed to receive robot state', throttle_duration_sec=1.0)
            return

        # difference between current robot pose and reference pose
        current_pos = PyKDL.Vector(
            state.cartesian.pos.x * 0.001,
            state.cartesian.pos.y * 0.001,
            state.cartesian.pos.z * 0.001
        )
        
        diff = self.H_0_N_robot_reference.p - current_pos
        
        # feefback
        force_x = diff.x() * 1000.0 * self.feedback_gain
        force_y = diff.y() * 1000.0 * self.feedback_gain
        force_z = diff.z() * 1000.0 * self.feedback_gain
        
        feedback_msg = JointState()
        feedback_msg.header.stamp = self.get_clock().now().to_msg()
        feedback_msg.effort = [force_x, force_y, force_z]
        
        self.feedback_pub.publish(feedback_msg)
        
        self.get_logger().debug(f'Feedback published: [{force_x:.3f}, {force_y:.3f}, {force_z:.3f}] N')

def main(args=None):
    rclpy.init(args=args)
    node = CartesianToHapticFeedback()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()