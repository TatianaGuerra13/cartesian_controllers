import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_publisher')

        self.publisher_ = self.create_publisher(Float64MultiArray, '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput', 10)

        timer_period = 0.02  # 50hz, try with the same frequency of decoder
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.start_time = time.time()
        self.freq = 0.2  # Hz

    def timer_callback(self):
        elapsed_time = time.time() - self.start_time

        msg = Float64MultiArray()

        amp = 0.1  # Ampiezza massima
        vx = amp * np.sin(2 * np.pi * self.freq * elapsed_time)
        vy = amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi/2)
        vz = amp * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi)

        wx = 0.05 * np.sin(2 * np.pi * self.freq * elapsed_time)
        wy = 0.05 * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi/2)
        wz = 0.05 * np.sin(2 * np.pi * self.freq * elapsed_time + np.pi)

        msg.data = [vx, vy, vz, wx, wy, wz]

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()