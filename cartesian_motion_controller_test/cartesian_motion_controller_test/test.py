import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import time

class Test(Node):
    def __init__(self):
        super().__init__('test')

        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput', 
            10
        )

        self.vx = 0.5 
        self.amp = 1.0      
        self.freq = 0.1     
        self.start = time.time()
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        t = time.time() - self.start
        vy = self.amp * math.sin(2 * math.pi * self.freq * t)
        vz = self.amp * math.sin(2 * math.pi * self.freq * t)
        msg = Float64MultiArray()
        msg.data = [
            self.vx,  # X velocity
            vy,#Y velocity
            vz,#Z velocity
            0.0,      # Angular X
            0.0,      # Angular Y
            0.0       # Angular Z
        ]
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = Test()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()