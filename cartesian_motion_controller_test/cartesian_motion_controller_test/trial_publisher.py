import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory
import csv

class TrialPublisher(Node):
    def __init__(self):
        super().__init__('trial_publisher')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput',
            10
        )

        try:
            package_share_dir = get_package_share_directory('cartesian_motion_controller_test')
            csv_path = os.path.join(package_share_dir, 'data', 'velocity_no_presentation_with_trials.csv')

            self.data = []

            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    self.data.append([
                        float(row['Var1']),
                        float(row['Var2']),
                        float(row['Var3']),
                        float(row['Var4']),
                        float(row['Var5']),
                        float(row['Var6'])
                    ])
                    if row['phase_label'] == 'SnapTo 2':  # analize until grasp
                        break

            self.get_logger().info(f'Loaded {len(self.data)} rows from first trial.')

        except Exception as e:
            self.get_logger().error(f'Could not read CSV: {e}')
            self.data = []

        self.index = 0
        self.dt = 0.0083  #  50 Hz
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
    node = TrialPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
