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
        self.object_vector_publisher = self.create_publisher(Point, '/object_vector', 10)

        # theta is the angle of the object
        self.theta = 0.0
        self.bool_detect = False


    ## Not sure if we need this algorithm
    # def Split_and_Merge(self, threshold):
    #     if (distance > threshold):
    #         P1 = Split_and_Merge(P[:ind+1,:],threshold) # split and merge left array
    #         P2 = Split_and_Merge(P[ind:,:],threshold) # split and merge right array
    #         # there are 2 "d" points, so exlude 1 (for example from 1st array)
    #         points = np.vstack((P1[:-1,:],P2))
    #     else:
    #         points = np.vstack((P[0,:],P[-1,:]))
    # return points


    def lidar_callback(self, posinfo):
        """
        LIDAR coordinate: counterclockwise = + theta
        """

        angle_min = posinfo.angle_min
        angle_inc = posinfo.angle_increment

        # Define the range in front of the camera that Turtlebot can see
        # roi_min = (-30) * (pi / 180)
        # roi_max = 30 * (pi / 180)

        # if roi_min <= self.theta and roi_max >= self.theta:
        ranges = np.array(posinfo.ranges)
        ranges = ranges[~np.isnan(ranges)]
        
        n = len(ranges)
        dist_min = np.min(ranges)
        dist_min_ind = np.argmin(ranges)
        dist_desired = 0.2
        epsilon = 0.05

        if dist_min <= dist_desired:
            """
            There is something detected! AO + FW!
            """
            self.bool_detect = True
            self.theta = angle_min + dist_min_ind * angle_inc
            
            # Beware of angle!
            # effective range of angle: -pi <= angle <= pi
            if self.theta > pi:
                self.theta = self.theta -  (2 * pi)

            pos = Twist()
            pos.linear.x = float(dist_min)
            pos.angular.z = self.theta
            self.object_position_publisher.publish(pos)

            # Turn the position info into vector
            obj_vector = Point()
            obj_vector.x = pos.linear.x * cos(pos.angular.z)
            obj_vector.y = pos.linear.x * sin(pos.angular.z)
            obj_vector.z = 0.0
            print("vector: x = ", obj_vector.x, "y = ", obj_vector.y)
            
            self.object_vector_publisher.publish(obj_vector)

        else:
            self.bool_detect = False
            self.theta = 0.0
            pos = Twist()
            pos.linear.x = 0.0
            pos.angular.z = 0.0
            pos.linear.z = -1.0
            self.object_position_publisher.publish(pos)

            # Turn the position info into vector
            obj_vector = Point()
            obj_vector.x = 0.0
            obj_vector.y = 0.0
            obj_vector.z = -1.0
            print("vector: x = ", obj_vector.x, "y = ", obj_vector.y)
            
            self.object_vector_publisher.publish(obj_vector)


def main(args=None):
    rclpy.init(args=args)
    get_object_range = GetRange()
    rclpy.spin(get_object_range)
    get_object_range.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
