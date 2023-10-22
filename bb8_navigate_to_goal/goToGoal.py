import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
import math

# Define the way points
way_point = np.array([[1.5,0], [1.5,1.4], [0,1.4]])
l = 0.1

class GoToGoal(Node):
    def __init__(self):
        super().__init__('go_to_goal')
        # State (for the update_Odometry code)
        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            image_qos_profile) # or 1?
        self.odom_sub  # prevent unused variable warning

        self._object_vector_subscriber = self.create_subscription(
            Twist,
            '/object_vector',
            self.object_callback,
            image_qos_profile)
        self._object_position_subscriber

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store the information for updating
        self.way_point_ind = 0



    def odom_callback(self, data):
        self.update_Odometry(data)


    def object_callback(self, pos):
        """
        Input is the vector of the obsatcle.
        Control the motor according to the vector.
        """


    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position

        # Orientation uses the quaternion aprametrization.
        # To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

        if self.Init:
            # The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])

        # We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))


    def GTG(self, xr, yr, angle):
        wayp_x, wayp_y = way_point[self.way_point_ind][:]
        ux = wayp_x - xr
        uy = wayp_y - yr

        rot_mtx = np.matrix([[np.cos(angle), np.sin(angle)],[-np.sin(angle), np.cos(angle)]])
        v, w = np.matrix([[1, 0], [0, 1/l]]) * rot_mtx * np.array([[ux], [uy]])
        
        return (v, w)

    def AO(self, xr, yr, angle):
        if True:
            pass
        else:
            return True



def main(args=None):
    rclpy.init(args=args)
    go_to_goal = GoToGoal()
    rclpy.spin(go_to_goal)
    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
