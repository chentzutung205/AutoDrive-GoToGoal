import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

import numpy as np
from math import pi

from rclpy.duration import Duration
import time


# Define the way points
way_point = np.array([[1.7, 0.0], [1.7, 1.5], [0, 1.5]])

# Define controllers' parameters
K_GTG = 1.0
K_AO = -1.0
L = 0.1 # What is the result of changing L?

dist_safety = 0.3
epsilon = 0.05


class BehaviorControl(Node):
    def __init__(self):
        super().__init__('behavior_control')
        
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
            image_qos_profile)
        self.odom_subscriber  # prevent unused variable warning

        self._object_position_subscriber = self.create_subscription(
            Twist,
            '/distance_and_angle',
            self.object_callback,
            image_qos_profile)
        self._object_position_subscriber

        self._object_vector_subscriber = self.create_subscription(
            Point,
            '/object_vector',
            self.vector_callback,
            image_qos_profile)
        self._object_vector_subscriber

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # To record at which way_point the Turtlebot is
        self.way_point_ind = 0
        
        self.bool_AO = False
        self.x_obs = 0.0
        self.y_obs = 0.0
        self.distance_obj = 1.0


    def odom_callback(self, data):
        self.update_Odometry(data)


    def object_callback(self, pos):
        """
        Input is the position info (Twist) of the object.
        """
        if pos.linear.z != -1.0:
            # Obstacle is detected! AO!
            self.distance_obj = pos.linear.x
            self.angle_obj = pos.angular.z


    def vector_callback(self, vector):
        """
        Input is the vector (Point) of the object w.r.t the robot.
        Control the motor accordingly!
        """
        if vector.z == -1.0:
            # Nothing is detected! GTG!
            self.bool_AO = False
        else:
            # Obstacle is detected! AO!
            self.bool_AO = True
            self.x_obs = vector.x
            self.y_obs = vector.y


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
            
            # This rotation matrix rotates clockwise
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        
        # This rotation matrix rotates clockwise
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
        
        # We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))

        if self.distance_obj <= dist_safety:
            u_AO = self.AO_control(self.globalPos.x, self.globalPos.y, self.globalAng)
            v = u_AO['v']
            w = u_AO['w']
            velocity = Twist()
            velocity.linear.x = float(v)
            velocity.angular.z = float(w)
            self.velocity_publisher.publish(velocity)
        
        u_GTG = self.GTG_control(self.globalPos.x, self.globalPos.y, self.globalAng)
        v = u_GTG['v']
        w = u_GTG['w']
        velocity = Twist()
        velocity.linear.x = float(v)
        velocity.angular.z = float(w)
        self.velocity_publisher.publish(velocity)

        
    def GTG_state(self):
        if self.way_point_ind == 0:
            self.way_point_ind = 1
            print("State 1!")
        elif self.way_point_ind == 1:
            self.way_point_ind = 2
            print("State 2!")
        elif self.way_point_ind == 2:
            print("Stop!")
            stop_velocity = Twist()
            self.velocity_publisher.publish(stop_velocity)


    def GTG_control(self, xr, yr, angle):
        wayp_x, wayp_y = way_point[self.way_point_ind][:]
        # print("wp_x: ", wayp_x, "wp_y: ", wayp_y)
        ux = K_GTG * (wayp_x - xr)
        uy = K_GTG * (wayp_y - yr)
        u = np.array([[ux], [uy]])

        wp_gap = way_point[self.way_point_ind] - np.array([xr, yr])
        print("wp_gap: ", wp_gap)
        
        if 0.015 > wp_gap[0] and wp_gap[0] > -0.015:
            if 0.02 > wp_gap[1] and wp_gap[1] > -0.02:
                print("Achieve goal!")
                time.sleep(10)
                self.GTG_state()

                if self.way_point_ind == 1:
                    print("Now wp_ind = 1!")
                elif self.way_point_ind == 2:
                    print("Now wp_ind = 2!")


        # The rotation matrix rotates clockwise
        rot_mtx = np.matrix([[np.cos(angle), np.sin(angle)],[-np.sin(angle), np.cos(angle)]])
        u_GTG = np.matrix([[1, 0], [0, 1/L]]) * rot_mtx * u

        v = u_GTG[0][0]
        w = u_GTG[1][0]
        
        # Limit robot's velocities
        if v > 0.22:
            v = 0.22
        elif v < -0.22:
            v = -0.22
        else:
            v = v

        if w > 2.84:
            w = 2.84
        elif w < -2.84:
            w = -2.84
        else:
            w = w

        print("v: ", v, "w: ", w)
        return {'v': v, 'w': w}

    def AO_control(self, xr, yr, angle):
        ux = K_AO * (self.x_obs - xr)
        uy = K_AO * (self.y_obs - yr)
        u = np.array([[ux], [uy]])

        # if self.dist_obj <= dist_safety and self.dist_obj > dist_safety - epsilon:
        #     print("It should follow the wall!")
        #     # Follow the wall!
        #     rot_c = np.matrix([[np.cos(pi/2), np.sin(pi/2)],[-np.sin(pi/2), np.cos(pi/2)]])
        #     rot_cc = np.matrix([[np.cos(-pi/2), np.sin(-pi/2)],[-np.sin(-pi/2), np.cos(-pi/2)]])
        #     u_fw_c = rot_c * u
        #     u_fw_cc = rot_cc * u

            # Let the goal decide our direction
            # dot_c = np.inner(np.reshape(u))
            # dot_cc = 
            

        # The rotation matrix rotates clockwise
        rot_mtx = np.matrix([[np.cos(-angle), np.sin(-angle)],[-np.sin(-angle), np.cos(-angle)]])
        u_AO = np.matrix([[1, 0], [0, 1/L]]) * rot_mtx * u

        v = u_AO[0][0]
        w = u_AO[1][0]
        
        # Limit robot's velocities
        if v > 0.22:
            v = 0.22
        elif v < -0.22:
            v = -0.22
        else:
            v = v

        if w > 2.84:
            w = 2.84
        elif w < -2.84:
            w = -2.84
        else:
            w = w
        
        print("v: ", v, "w: ", w)
        return {'v': v, 'w': w}


def main(args=None):
    rclpy.init(args=args)
    behavior_control = BehaviorControl()
    rclpy.spin(behavior_control)
    behavior_control.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
