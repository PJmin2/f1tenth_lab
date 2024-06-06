import rclpy
import numpy as np

from rclpy.node import Node
from numpy import linalg as LA
from tf_transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

file = open('waypoint.csv', 'w')

class WaypointLogger(Node):

    def __init__(self):
        super().__init__('waypoint_logger')

        self.subscription = self.create_subscription(Odometry, '/odom', self.save_waypoint, 10)


    def save_waypoint(self, data: Odometry):
        quaternion = np.array([data.pose.pose.orientation.x,
                               data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z,
                               data.pose.pose.orientation.w])

        euler = euler_from_quaternion(quaternion)
        speed = LA.norm(np.array([data.twist.twist.linear.x,
                                  data.twist.twist.linear.y,
                                  data.twist.twist.linear.z]),2)
        if data.twist.twist.linear.x>0.:
            print(data.twist.twist.linear.x)

        file.write('%f, %f, %f, %f\n' % (data.pose.pose.position.x,
                                         data.pose.pose.position.y,
                                         euler[2],
                                         speed))


def shutdown():
    file.close()
    print('Goodbye')


def main(args=None):
    print('Saving waypoints...')
    rclpy.init(args=args)
    waypoint_logger = WaypointLogger()
    try:
        rclpy.spin(waypoint_logger)
    except KeyboardInterrupt:
        waypoint_logger.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
       shutdown()
       waypoint_logger.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
    main()

