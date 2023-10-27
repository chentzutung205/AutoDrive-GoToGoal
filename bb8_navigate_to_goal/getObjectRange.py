import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from math import pi, cos, sin


class GetRange(Node):
    def __init__(self):
        super().__init__('get_object_range')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self._lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, image_qos_profile)
        self._lidar_subscriber

        self.object_position_publisher = self.create_publisher(Twist, '/distance_and_angle', 10)

        # theta is the angle of the object
        self.theta = 0.0
        self.bool_detect = False


    def lidar_callback(self, posinfo):
        """
        LIDAR coordinate: counterclockwise = + theta
        """

        angle_min = posinfo.angle_min
        angle_inc = posinfo.angle_increment

        ranges = np.array(posinfo.ranges)
        ranges = ranges[~np.isnan(ranges)]
        
        dist_min = np.min(ranges)
        dist_min_ind = np.argmin(ranges)

        self.theta = angle_min + dist_min_ind * angle_inc
        # Beware of angle!
        # effective range of angle: -pi <= angle <= pi
        if self.theta > pi:
                self.theta = self.theta -  (2 * pi)

        pos = Twist()
        pos.linear.x = float(dist_min)
        pos.angular.z = self.theta

        self.object_position_publisher.publish(pos)
        

def main(args=None):
    rclpy.init(args=args)
    get_object_range = GetRange()
    rclpy.spin(get_object_range)
    get_object_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
