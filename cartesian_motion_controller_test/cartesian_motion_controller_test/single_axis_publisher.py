import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class SingleAxisPublisher(Node):
    def __init__(self):
        super().__init__('single_axis_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput', 
            10
        )

        timer_period = 0.002  # 500 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.start_time = time.time()
        self.freq = 0.2  # Hz
        self.velocity_amplitude = 0.5  # m/s

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time

        # Direct velocity command without displacement limits
        vx = self.velocity_amplitude * np.sin(2 * np.pi * self.freq * elapsed_time)

        # Create message with only X velocity
        msg = Float64MultiArray()
        msg.data = [
            vx,   # X velocity
            0.0,  # Y velocity
            0.0,  # Z velocity
            0.0,  # Angular X
            0.0,  # Angular Y
            0.0   # Angular Z
        ]

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = SingleAxisPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()