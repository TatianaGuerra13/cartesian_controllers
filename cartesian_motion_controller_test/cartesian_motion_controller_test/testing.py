import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import os
from ament_index_python.packages import get_package_share_directory
import csv
import math


class Testing(Node):
    def __init__(self):
        super().__init__('testing')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/cartesian_motion_controller_silvestro/CartesianMotionControllerInput',
            10
        )

        try:
            package_share_dir = get_package_share_directory('cartesian_motion_controller_test')
            csv_path = os.path.join(package_share_dir, 'data', 'actualvel_no_presentation_with_trials.csv')

            self.data = []

            with open(csv_path, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        vx = float(row['pos1'])
                    except ValueError: 
                        vx = 0.0
                    if math.isnan(vx):
                        vx = 0.0
                        vx*=10.0  # Scale the velocity if needed
                    self.data.append([
                        vx,    # vx
                        0.0,   # vy
                        0.0,   # vz
                        0.0,   # wx
                        0.0,   # wy
                        0.0    # wz
                    ])
                    if row['phase_label'] == 'Release 54':  # analize until grasp
                        break

            self.get_logger().info(f'Loaded {len(self.data)} rows from first trial.')

        except Exception as e:
            self.get_logger().error(f'Could not read CSV: {e}')
            self.data = []

        self.index = 0
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.publish_next)

    def publish_next(self):
        # Se ci sono ancora dati, pubblica la riga corrente
        if self.index < len(self.data):
            row = self.data[self.index]
            self.index += 1
        else:
            # Se hai finito i dati, pubblica sempre l'ultima riga
            row = self.data[-1]

        msg = Float64MultiArray()
        msg.data = row
        self.publisher_.publish(msg)

        self.get_logger().info(f'[{self.index-1 if self.index<=len(self.data) else len(self.data)-1}] Published: {["{:.3f}".format(x) for x in row]}')
            


def main(args=None):
    rclpy.init(args=args)
    node = Testing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
