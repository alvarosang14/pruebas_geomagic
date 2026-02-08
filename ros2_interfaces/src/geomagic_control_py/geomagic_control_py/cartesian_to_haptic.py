import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_srvs.srv import SetBool
from std_msgs.msg import Wrench

NODE = 'cartesian_to_haptic'
HAPTIC_YARP_DEVICE_NODE_NAME = 'haptic_yarp_device'

class CartesianToHaptic(Node):

    def __init__(self):
        super().__init__(NODE)
        self.publish_cartesian_create()
        if not self.set_feedback_mode():
            self.get_logger().error("Failed to configure cartesian control server")
            return
        self.haptic_subscriber_create()
        self.get_logger().info('Cartesian to Haptic Feedback Node started')

    # -------------------------------------------------------------------------------------------
    def publish_cartesian_create(self):
        self.m_haptic_feedback_pub = self.create_publisher(String, HAPTIC_YARP_DEVICE_NODE_NAME + '/feedback', 10)

    def set_feedback_mode(self):
        self.client_set_feedback_mode = self.create_client(SetBool, 'set_feedback_mode')
        
        request = SetBool.Request()
        request.data = False
        
        while not self.client_set_feedback_mode.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                self.get_logger().error('Interrupted while waiting for the service. Exiting.')
                return False
            self.get_logger().info('service not available, waiting again...')
        
        future = self.client_set_feedback_mode.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            self.get_logger().info('Feedback mode set successfully')
            return future.result().success
        else:
            self.get_logger().error('Failed to call service set_feedback_mode')
            return False
    
    def haptic_subscriber_create(self):
        self.create_subscription(
            Wrench,
            '/state',
            self.haptic_state_callback,
            10
        )

    # -------------------------------------------------------------------------------------------
    def haptic_state_callback(self, msg):
        self.publish_haptic_feedback(msg)

    # -------------------------------------------------------------------------------------------
    def publish_haptic_feedback(self, feedback):
        self.m_haptic_feedback_pub.publish(feedback)
        self.get_logger().info(f"Published haptic feedback: {feedback.force.x}, {feedback.force.y}, {feedback.force.z}")

def main(args=None):
    try:
        with rclpy.init(args=args):
            rclpy.spin(CartesianToHaptic())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()