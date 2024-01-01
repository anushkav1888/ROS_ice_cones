#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class BicycleMobileRobot:
    def __init__(self):
        rospy.init_node('main_controller', anonymous=True)
        self.L = 0
        self.active_waypoint = np.zeros(2)
        self.radius_icecone = 0
        self.start_point = np.zeros(2)
        self.end_point = np.zeros(2)
        self.max_range = None
        self.pose = []
        self.ranges = []
        self.angles = []
        self.target_goal = [50,50] 

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=10)
        rospy.Subscriber('/pf/pose/odom',Odometry,self.RobotPose)
        pub = rospy.Publisher('/racecar/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
        velocity_msg = AckermannDriveStamped()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if len(self.pose) == 0:
                continue

            if len(self.ranges) == 0:
                continue

            [velocity_msg.drive.speed, velocity_msg.drive.steering_angle_velocity] = self.ipc_navigate(self.target_goal)
            pub.publish(velocity_msg)
        rate.sleep()

    def lidar_callback(self, msg):
        print("range funcc")
        self.ranges = msg.ranges
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        print(self.ranges[2])
        print(self.angles[2])

    def RobotPose(self,data):
        print("pose func was called")
        self.pose = [data.pose.pose.position.x,data.pose.pose.position.y,euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]]
        print(self.pose)

    def euclidean_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) 
    
    def R_optimizer(self, int_goal_x, int_goal_y, Robot_x, Robot_y, T, R_max, angles, ranges):
        angle_diff = angles[T] - np.arctan2(int_goal_y, int_goal_x)

        if np.cos(angle_diff) > 0:
            distance_to_goal = self.euclidean_distance(int_goal_x, int_goal_y, Robot_x, Robot_y)
            if distance_to_goal * np.cos(angle_diff) >= R_max:
                R_opt = R_max
            else:
                R_opt = distance_to_goal * np.cos(angle_diff)
        else:
            R_opt = 0

        return R_opt
    


    def sensor_data_process(self, ranges, T, pose):
        I = 0
        x_ref = ranges[T]
        N = len(ranges)
        del_theta = 2 * np.pi / N
        
        pose = np.mod(pose, 2 * np.pi)
        robot_index = int(np.round(pose / del_theta) + 1)
        if robot_index > N:
            robot_index = 1
        
        theta = min(abs(T - robot_index) * del_theta, 2 * np.pi - abs(T - robot_index) * del_theta)
        
        if theta >= np.pi / 2:
            if pose <= np.pi:
                robot_index = int(np.round((pose + np.pi) / del_theta) + 1)
                if robot_index > N:
                    robot_index = 1
            if pose > np.pi:
                robot_index = int(np.round((pose - np.pi) / del_theta) + 1)
        
            theta = min(abs(T - robot_index) * del_theta, 2 * np.pi - abs(T - robot_index) * del_theta)
            if theta == abs(T - robot_index) * del_theta:
                a = 2 * T - robot_index
                if a > N:
                    a = a - N
                end_idx = int(min(a, robot_index))
                start_idx = int(max(a, robot_index))
                for i in range(start_idx, N):

                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - start_idx) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + term)
                        if np.imag(x_ref) != 0:
                            a = 4
                for i in range(0, end_idx):
                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - 1) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - 1) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - 1) * del_theta) + term)
                        if np.imag(x_ref) != 0:
                            a = 5
            elif a < 1:
                a = a + N
                end_idx = min(a, robot_index)
                start_idx = max(a, robot_index)
                for i in range(start_idx, N):
                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - start_idx) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + term)
                        if np.imag(x_ref) != 0:
                            a = 6
                for i in range(0, end_idx):
                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - 1) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - 1) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - 1) * del_theta) + term)
                        if np.imag(x_ref) != 0:
                            a = 7
            else:
                start_idx = min(a, robot_index)
                end_idx = max(a, robot_index)
                for i in range(start_idx, end_idx):
                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - start_idx) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + term)
 
            if theta == 2 * np.pi - abs(T - robot_index) * del_theta:
                p = min(T, robot_index)
                q = max(T, robot_index)
                diff = p - q + N - 1
                if robot_index == p:
                    a = T - diff - 1
                if robot_index == q:
                    a = T + diff + 1
                end_idx = min(a, robot_index)
                start_idx = max(a, robot_index)
                for i in range(start_idx, N):
                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - start_idx) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + term)
                        if np.imag(x_ref) != 0:
                            a = 2
                for i in range(0, end_idx):
                    term = (-np.cos(theta) ** 2 + (np.cos(theta - (i - 1) * del_theta)) ** 2) ** 0.5
                    if ranges[i] < x_ref * (np.cos(theta - (i - 1) * del_theta) + term):
                        x_ref = ranges[i] / (np.cos(theta - (i - 1) * del_theta) + term)
                        if np.imag(x_ref) != 0:
                            a = 3
        return x_ref, I

                                                                    
    def ipc_navigate(self, target_goal):
  
        R_max = np.zeros(len(self.ranges))
        R_opt = np.zeros(len(self.ranges))
        D_check = np.zeros(len(self.ranges))

        for T in range(len(self.ranges)):
            R_max[T], _ = self.sensor_data_process(self.ranges, T, self.pose[2])
            R_opt[T] = self.R_optimizer(self.target_goal[0], self.target_goal[1], self.pose[0], self.pose[1], T, R_max[T],
                                   self.angles, self.ranges)
            D_check[T] = self.euclidean_distance(target_goal[0], target_goal[1],
                                            self.pose[0] + R_opt[T] * np.cos(self.angles[T]),
                                            self.pose[1] + R_opt[T] * np.sin(self.angles[T]))

        i_best = np.argmin(D_check)

        way_x_opt = self.pose[0] + R_opt[i_best] * np.cos(self.angles[i_best])
        way_y_opt = self.pose[1] + R_opt[i_best] * np.sin(self.angles[i_best])
        self.active_waypoint = np.array([way_x_opt, way_y_opt])

        rel_bearing = self.pose[2] - np.arctan2(self.pose[1] - way_y_opt, self.pose[0] - way_x_opt)

        if rel_bearing >= np.pi:
            rel_bearing = rel_bearing - 2 * np.pi
        elif rel_bearing <= -np.pi:
            rel_bearing = rel_bearing + 2 * np.pi

        if rel_bearing > np.pi / 2:
            sigma = rel_bearing - np.pi
        elif rel_bearing < -np.pi / 2:
            sigma = rel_bearing + np.pi
        else:
            sigma = rel_bearing

        K_1 = 10
        K_2 = 5

        v = -K_1 * np.tanh(R_opt[i_best]) * np.sign(np.cos(rel_bearing))
        omega = -K_2 * np.sign(sigma) * np.abs(sigma) ** 0.5 + (v / R_opt[i_best]) * np.sin(rel_bearing)

        self.controls = np.array([v, omega])
        return self.controls   
    
if __name__ == '__main__':    
    try:
        time.sleep(5)
        BicycleMobileRobot()
    except rospy.ROSInterruptException:
        pass
  






