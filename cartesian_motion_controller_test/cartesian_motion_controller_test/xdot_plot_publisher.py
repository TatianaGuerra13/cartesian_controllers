import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped

class XDotPlotPublisher(Node):
    def __init__(self):
        super().__init__('xdot_plot_publisher')

        # Subscribe al comando di input (xdot dato al controller)
        self.subscription_input = self.create_subscription(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput',
            self.input_callback,
            10)

        # Subscribe alla velocità effettiva pubblicata dal controller
        self.subscription_twist = self.create_subscription(
            TwistStamped,
            '/cartesian_motion_controller_silvestro/current_twist',
            self.twist_callback,
            10)

        # Publisher sui topic che useremo per PlotJuggler
        self.pub_input = self.create_publisher(Float64MultiArray, '/xdot_input', 10)
        self.pub_measured = self.create_publisher(Float64MultiArray, '/xdot_measured', 10)

    def input_callback(self, msg):
        self.pub_input.publish(msg)

    def twist_callback(self, msg):
        out_msg = Float64MultiArray()
        out_msg.data = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
            msg.twist.angular.x,
            msg.twist.angular.y,
            msg.twist.angular.z
        ]
        self.pub_measured.publish(out_msg)

def main():
    rclpy.init()
    node = XDotPlotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
