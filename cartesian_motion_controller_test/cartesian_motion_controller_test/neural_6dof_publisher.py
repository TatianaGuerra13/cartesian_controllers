import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory

class Neural6DOFPublisher(Node):
    def __init__(self):
        super().__init__('neural_6dof_publisher')

        self.publisher_ = self.create_publisher(Float64MultiArray, '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput', 10)
        
        # Load CSV data
        self.data = []
        try:
            package_share_dir = get_package_share_directory('cartesian_motion_controller_test')  
            csv_path = os.path.join(package_share_dir, 'data', 'velocity_6gdl_grasp_carry.csv')
            
            with open(csv_path, 'r') as f:
                next(f, None) # Skip header line
                for line in f:
                    values = line.strip().split(',')
                    if len(values) == 6:
                        self.data.append([float(x) for x in values])
            self.get_logger().info(f'Loaded {len(self.data)} rows from CSV.')
        except Exception as e:
            self.get_logger().error(f'Could not read CSV: {e}')
            self.data = []

        self.index = 0
        self.dt = 0.01  # 100 Hz ??
        self.timer = self.create_timer(self.dt, self.publish_next)

    def publish_next(self):
        if self.index >= len(self.data):
            self.get_logger().info('Finished publishing CSV.')
            self.timer.cancel()
            return

        row = self.data[self.index]
        msg = Float64MultiArray()
        msg.data = row
        self.publisher_.publish(msg)

        self.get_logger().info(f'[{self.index}] Published: {["{:.3f}".format(x) for x in row]}')
        self.index += 1

def main(args=None):
    rclpy.init(args=args)
    node = Neural6DOFPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()