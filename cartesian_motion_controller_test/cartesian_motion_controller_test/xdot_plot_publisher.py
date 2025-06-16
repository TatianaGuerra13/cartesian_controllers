import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped

class XDotPlotPublisher(Node):
    def __init__(self):
        super().__init__('xdot_plot_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/cartesian_motion_controller/CartesianMotionControllerInput', 
            self.input_callback,
            10)

        self_subscription = self.create_subscription(
            TwistStamped,
            '/cartesian_motion_controller/current_twist',
            self.twist_callback,
            10)

        self.self.pub_input = self.create_publisher(Float64MultiArray, '/xdot_input', 10)
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
    node = XdotPlotPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

