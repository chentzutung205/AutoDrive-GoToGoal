import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Twist
import math
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
import time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import asyncio

class goToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal')

        image_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
		    history=QoSHistoryPolicy.KEEP_LAST,
		    durability=QoSDurabilityPolicy.VOLATILE,
		    depth=1
		)

        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0

        self.globalPos = Point()

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            image_qos_profile)
        self.odom_sub  # prevent unused variable warning


        self.obstacle_sub = self.create_subscription(
            Twist,
            '/distance_and_angle',
            self.obstacle_callback,
            image_qos_profile)
        self.obstacle_sub


        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.waypoints = []
        self.readWayPoints()
        self.glb_goal = np.zeros(2)
        self.glb_bot = np.zeros(2)
        self.robot_vel_ang = np.zeros(2)
        self.glb_obstacle = np.array([1.6, 0.7])

        self.goal_1_reached = False
        self.goal_2_reached = False
        self.goal_3_reached = False

        self.mode = 'goToGoal'
        self.distance_limit = 0.18
        self.epsilon = 0.05

        self.movement_robot = Twist()
        self.movement_robot.linear.x = 0.0
        self.movement_robot.angular.z = 0.0

        self.theta_desired_goal = 0
        self.theta_desired_fwc = 0
        self.theta_desired_fwcc = 0
        self.count = 0
        self.switch_count = 0
        self.globalAng = 0
        self.globalObstacle = Point()

        self.timer = self.create_timer(0.1, self.operate)


    def readWayPoints(self):
        # Read WayPoints and store
        waypoints = np.array([[1.65, 0], [1.65, 1.55], [0, 1.55]])

        wayPoint1 = Twist()
        wayPoint1.linear.x = waypoints[0][0]
        wayPoint1.linear.y = waypoints[0][1]
        self.waypoints.append(wayPoint1)

        wayPoint2 = Twist()
        wayPoint2.linear.x = waypoints[1][0]
        wayPoint2.linear.y = waypoints[1][1]
        self.waypoints.append(wayPoint2)

        wayPoint3 = Twist()
        wayPoint3.linear.x = waypoints[2][0]
        wayPoint3.linear.y = waypoints[2][1]
        self.waypoints.append(wayPoint3)


    def odom_callback(self, data):
        self.update_Odometry(data)

    def update_Odometry(self, Odom):
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
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)], [-np.sin(self.Init_ang), np.cos(self.Init_ang)]])

        # We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang

        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x, self.globalPos.y, self.globalAng))

        self.glb_bot[0] = self.globalPos.x
        self.glb_bot[1] = self.globalPos.y


    def operate(self):
        # Tell which goal points is reached and change current goal
        if (self.goal_1_reached == False) and (self.goal_2_reached == False) and (self.goal_3_reached == False):
            self.glb_goal[0] = (self.waypoints[0]).linear.x
            self.glb_goal[1] = (self.waypoints[0]).linear.y
            self.goToGoal_controller(1)

        if (self.goal_1_reached == True) and (self.goal_2_reached == False) and (self.goal_3_reached == False):
            #print('goal 1 reached')
            self.glb_goal[0] = (self.waypoints[1]).linear.x
            self.glb_goal[1] = (self.waypoints[1]).linear.y
            self.goToGoal_controller(2)

        if (self.goal_1_reached == True) and (self.goal_2_reached == True) and (self.goal_3_reached == False):
            #print('goal 2 reached')
            self.glb_goal[0] = (self.waypoints[2]).linear.x
            self.glb_goal[1] = (self.waypoints[2]).linear.y
            self.goToGoal_controller(3)

        # print("Next Target: ", self.glb_goal)

        self.avoidObstacle_controller()
        self.followWallC_controller()
        self.followWallCC_controller()
        self.switch_logic()
        self.check_progress()
        self.velocity_set()
        self.movement_robot.linear.x = self.robot_vel_ang[0]
        self.movement_robot.angular.z = self.robot_vel_ang[1]
        self.cmd_vel_pub.publish(self.movement_robot)

            # 每0.1秒调用一次self.operate
            #print(self.movement_robot)
            #print(self.glb_bot)


    def switch_logic(self):

        self.distance_to_object = np.linalg.norm(self.glb_bot - self.glb_obstacle)
        # print("obs_loc: ", self.glb_obstacle)
        # print("rob2obs: ", self.distance_to_object)
        # print(self.distance_limit-self.epsilon)

        if (self.mode == "goToGoal") and (self.distance_to_object <= (self.distance_limit)) and (self.u_GTG.T.dot(self.u_FWC) > 0):
            self.glb_robot_switch = np.copy(self.glb_bot)
            self.mode = "followWallC"
            self.switch_time = self.get_clock().now()
            self.switch_count+=1

        if (self.mode == "followWallC") and (self.distance_to_object < (self.distance_limit - self.epsilon)):
            self.mode = "avoidObstacle"

        if (self.mode == "goToGoal") and (self.distance_to_object <= (self.distance_limit)) and (self.u_GTG.T.dot(self.u_FWCC) > 0):
            self.mode = "followWallCC"

        if (self.mode == "avoidObstacle") and (self.distance_to_object >= (self.distance_limit)) and (self.u_GTG.T.dot(self.u_FWC) > 0):
            self.glb_robot_switch = np.copy(self.glb_bot)
            self.mode = "followWallC"
            self.switch_time = self.get_clock().now()
            self.switch_count+=1

        if (self.mode == "avoidObstacle") and (self.distance_to_object >= (self.distance_limit)) and (self.u_GTG.T.dot(self.u_FWCC) > 0):
            self.glb_robot_switch = np.copy(self.glb_bot)
            self.mode = "followWallCC"
            self.switch_time = self.get_clock().now()
            self.switch_count+=1

        if (self.mode == "followWallCC") and (self.distance_to_object < (self.distance_limit - self.epsilon)):
            self.mode = "avoidObstacle"

        # if ((self.mode == "followWallC")or(self.mode == "followWallCC")) and (self.u_GTG.T.dot(self.u_AO) > 0) and\
        #          (np.linalg.norm(self.glb_bot - self.glb_goal) < (np.linalg.norm(self.glb_robot_switch - self.glb_goal) + 0.5)) and\
        #          (self.distance_to_object > (self.distance_limit+self.epsilon)) and ((self.get_clock().now()-self.switch_time)>= Duration(seconds=2)):
        #     self.mode = "goToGoal"

        if ((self.mode == "followWallC") or (self.mode == "followWallCC")) and (self.u_GTG.T.dot(self.u_AO) > 0) and (self.distance_to_object >= (self.distance_limit + self.epsilon)):
            self.mode = "goToGoal"

        print("Current Mode: ", self.mode)


    def obstacle_callback(self, obj_pos):
        # Transfer obstacle coordinate from local to global
        self.globalObstacle.x = self.globalPos.x + obj_pos.linear.x * np.cos(self.globalAng + np.deg2rad(obj_pos.angular.z))
        self.globalObstacle.y = self.globalPos.y + obj_pos.linear.x * np.sin(self.globalAng + np.deg2rad(obj_pos.angular.z))
        self.obstacle_dist = obj_pos.linear.x
        self.obstacle_orient = obj_pos.angular.z
        self.glb_obstacle[0] = self.globalObstacle.x
        self.glb_obstacle[1] = self.globalObstacle.y


    def goToGoal_controller(self, goal):
        kp_GTG = 0.8
        ki_GTG = 0
        kd_GTG = 0

        self.u_GTG = kp_GTG * (self.glb_goal - self.glb_bot)


    def avoidObstacle_controller(self):
        k_AO = 0.8
        self.u_AO = -k_AO * (self.glb_obstacle - self.glb_bot)

    def followWallC_controller(self):
        alpha = 0.2
        self.u_FWC = alpha * (self.rotationMatrix(-np.pi/2)).dot(self.u_AO)

    def followWallCC_controller(self):
        alpha = 0.2
        self.u_FWCC = alpha * (self.rotationMatrix(np.pi/2)).dot(self.u_AO)

    def rotationMatrix(self, theta):
        matrix = np.zeros((2,2))
        matrix[0][0] = np.cos(theta)
        matrix[0][1] = np.sin(theta)
        matrix[1][0] = -np.sin(theta)
        matrix[1][1] = np.cos(theta)

        return matrix

    def desired_heading(self):
        self.theta_desired_goal = np.arctan2((self.glb_goal[1] - self.glb_bot[1]), (self.glb_goal[0] - self.glb_bot[0]))
        self.theta_desired_fwc = np.arctan2(self.u_FWC[0], self.u_FWC[1])
        self.theta_desired_fwcc = np.arctan2(self.u_FWCC[0], self.u_FWCC[1])


    def velocity_set(self):
        self.desired_heading()
        self.k_p_angular = 1.2
        self.k_p_linear = 1

        if self.mode == "goToGoal":
            angle_error = self.theta_desired_goal - self.globalAng
            angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
            self.angular_velocity = self.k_p_angular * (angle_error)

            if abs(angle_error) <= np.deg2rad(10):
                self.linear_velocity =self.k_p_linear * np.linalg.norm(self.u_GTG)
            else:
                self.linear_velocity = 0

            if self.linear_velocity >= 0.2:
                self.linear_velocity = 0.15
            elif self.linear_velocity <= -0.2:
                self.linear_velocity = -0.15

            if self.angular_velocity >= 1.5:
                self.angular_velocity = 1.5
            elif self.angular_velocity <= -1.5:
                self.angular_velocity = -1.5

            self.robot_vel_ang[0] = self.linear_velocity
            self.robot_vel_ang[1] = self.angular_velocity

        elif self.mode == "avoidObstacle":
            self.angular_velocity = 0
            self.linear_velocity = -self.k_p_linear * np.linalg.norm(self.u_AO)

            if self.linear_velocity >= 0.2:
                self.linear_velocity = 0.15
            elif self.linear_velocity <= -0.2:
                self.linear_velocity = -0.15

            if self.angular_velocity >= 1.5:
                self.angular_velocity = 1.5
            elif self.angular_velocity <= -1.5:
                self.angular_velocity = -1.5

            self.robot_vel_ang[0] = self.linear_velocity
            self.robot_vel_ang[1] = self.angular_velocity

        elif self.mode == "followWallC":
            self.angular_velocity = 0.5 * self.k_p_angular * (self.theta_desired_fwc - self.globalAng)
            self.linear_velocity = self.k_p_linear * np.linalg.norm(self.u_FWC)

            if self.linear_velocity >= 0.2:
                self.linear_velocity = 0.15
            elif self.linear_velocity <= -0.2:
                self.linear_velocity = -0.15

            if self.angular_velocity >= 1.5:
                self.angular_velocity = 1.5
            elif self.angular_velocity <= -1.5:
                self.angular_velocity = -1.5

            self.robot_vel_ang[0] = self.linear_velocity
            self.robot_vel_ang[1] = self.angular_velocity

        elif self.mode == "followWallCC":
            self.angular_velocity = -0.5 * self.k_p_angular * (self.theta_desired_fwcc - self.globalAng)
            self.linear_velocity = self.k_p_linear * np.linalg.norm(self.u_FWCC)

            if self.linear_velocity >= 0.2:
                self.linear_velocity = 0.15
            elif self.linear_velocity <= -0.2:
                self.linear_velocity = -0.15

            if self.angular_velocity >= 1.5:
                self.angular_velocity = 1.5
            elif self.angular_velocity <= -1.5:
                self.angular_velocity = -1.5

            self.robot_vel_ang[0] = self.linear_velocity
            self.robot_vel_ang[1] = self.angular_velocity


    def check_progress(self):
        # Check if goal is reached
        if(self.glb_bot[0] >= 1.63) and (self.glb_bot[0] <= 1.65) and (self.glb_bot[1] >= -0.12) and (self.glb_bot[1] <= 0.12) and (self.count == 0):
            self.goal_1_reached = True
            self.movement_robot.linear.x = 0.0
            self.movement_robot.angular.z = 0.0
            self.cmd_vel_pub.publish(self.movement_robot)
            print("Reach the 1st goal!")
            time.sleep(10)
            self.count += 1

        if(self.glb_bot[0] >= 1.63) and (self.glb_bot[0] <= 1.67) and (self.glb_bot[1] >= 1.56) and (self.glb_bot[1] <= 1.58) and (self.count == 1):
            self.goal_2_reached = True
            self.movement_robot.linear.x = 0.0
            self.movement_robot.angular.z = 0.0
            self.cmd_vel_pub.publish(self.movement_robot)
            print("Reach the 2nd goal!")
            time.sleep(10)
            self.count += 1

        if(self.glb_bot[0] >= 0.10) and (self.glb_bot[0] <= 0.40) and (self.glb_bot[1] >= 1.45) and (self.glb_bot[1] <= 1.55) and (self.count == 2):
            self.goal_3_reached = True
            self.movement_robot.linear.x = 0.0
            self.movement_robot.angular.z = 0.0
            self.cmd_vel_pub.publish(self.movement_robot)
            print("Reach the FANAL goal!")
            time.sleep(10)
            self.count += 1


def main(args=None):
    rclpy.init(args=args)
    go_to_goal = goToGoal()
    rclpy.spin(go_to_goal)
    go_to_goal.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
