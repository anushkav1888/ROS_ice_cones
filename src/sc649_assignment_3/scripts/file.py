#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import time
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

class BicycleMobileRobot:
    def __init__(self):
        rospy.init_node('main_controller', anonymous=True)
        self.L = 0.40
        self.active_waypoint = np.zeros(2)
        self.radius_icecone = 0
        self.start_point = np.zeros(2)
        self.end_point = np.zeros(2)
        self.max_range = None
        self.pose = []
        self.transformedPose = []
        self.del_T = 2.7
        self.ranges = []
        self.angles = []
        start_time = time.time()
        self.speed = 0
        self.target_goal = [5, 0]
        self.linear_velocity_x = 0
        self.linear_velocity_y = 0
        self.angular_velocity_z = 0
        self.steer_angle = 0

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=10)
        rospy.Subscriber('/pf/pose/odom', Odometry, self.RobotPose)
        self.pub = rospy.Publisher('/racecar/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
        self.velocity_msg = AckermannDriveStamped()
        self.rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if len(self.pose) == 0:
                continue

            if len(self.ranges) == 0:
                continue

            [self.velocity_msg.drive.speed, self.velocity_msg.drive.steering_angle] = [10,np.pi/3]
            #[self.velocity_msg.drive.speed, self.velocity_msg.drive.steering_angle] = self.ipc_navigate(
             #   self.target_goal)
            self.pub.publish(self.velocity_msg)
            print(self.speed,self.velocity_msg.drive.speed,time.time() - start_time)
            loop_duration = time.time() - start_time
            #print(time.time())
            # Print the loop duration
            #print(f"Loop Time: {loop_duration:.6f} seconds")
            #start_time = time.time()
            self.rate.sleep()

    def lidar_callback(self, msg):
        self.ranges = msg.ranges
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))

    def RobotPose(self, data):
        yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                           data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y,yaw,10]
        self.linear_velocity_x = data.twist.twist.linear.x
        self.linear_velocity_y = data.twist.twist.linear.y
        self.angular_velocity_z = data.twist.twist.angular.z
        self.speed = np.sqrt((self.linear_velocity_x ) ** 2 + (self.linear_velocity_y) ** 2)

    def euclidean_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def angle_normalizer(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
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


    def sensor_data_process(self,ranges, T, pose):
        I = 0
        x_ref = ranges[T]  # initialize x_ref
        N = len(ranges)
        del_theta = 2 * np.pi / N

        pose = np.mod(pose, 2 * np.pi)
        pose = pose + 2 * np.pi * (pose < 0)
        robot_index = int(np.floor(pose / del_theta))   # the direction in which the robot is facing
        norm_R = []

        theta = min(abs(T - robot_index) * del_theta, 2 * np.pi - abs(T - robot_index) * del_theta)

        a = 2 * T - robot_index  # the other end of the ice cone, starts at robot_index and ends at a

        if theta < np.pi / 2 and theta > 0:
            if theta == 2 * np.pi - abs(T - robot_index) * del_theta:
                a = int(2 * T - robot_index)
                if a > N:
                    a = a - N
                if a < 0:
                    a = a + N
                end_idx = int(min(a, robot_index))
                start_idx = int(max(a, robot_index))

                for i in range(start_idx, N):  # moving till N and then to end_index which is in the first quadrant
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            print("33")
                            x_ref = np.real(x_ref)

                for i in range(0, end_idx):
                    if ranges[i] < x_ref * (np.cos(theta - (i + N - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i + N - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            print("44")
                            x_ref = np.real(x_ref)

            elif theta == abs(T - robot_index) * del_theta:
                a = int(2 * T - robot_index)
                if a < 0 or a >= N:
                    if a >= N:
                        a = a - N
                    if a < 0:
                        a = a + N

                    start_idx = int(max(a, robot_index))
                    end_idx = int(min(a, robot_index))
                    for i in range(start_idx, N):
                        if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                            x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                            if np.imag(x_ref) != 0:
                                print("68")
                                x_ref = np.real(x_ref)

                    for i in range(0, end_idx):
                        if ranges[i] < x_ref * (np.cos(theta - (i + N - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5):
                            x_ref = ranges[i] / (np.cos(theta - (i + N - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5)
                            if np.imag(x_ref) != 0:
                                print("80")
                                x_ref = np.real(x_ref)

                elif abs(T - robot_index) == abs(T - a):
                    start_idx = int(min(a, robot_index))
                    end_idx = int(max(a, robot_index))
                    for i in range(start_idx, end_idx):
                        if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                            x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) +
                                                    (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                            if np.imag(x_ref) != 0:
                                print("94")
                                x_ref = np.real(x_ref)

        if theta == np.pi / 2:
            for i in range(round(T - (len(ranges) - 2) / 4), round(T + (len(ranges) - 2) / 4)):  # spanning range
                eff_i = (i + len(ranges)) if i <= 0 else (i - len(ranges)) if i > len(ranges) else i
                norm_R.append(ranges[eff_i] / (2 * np.cos((T - i) * del_theta) + 0.0001))

            x_ref, I = min(norm_R)

        if np.pi / 2 < theta < np.pi:
            pose = pose - np.pi
            pose = np.mod(pose, 2 * np.pi)
            pose = pose + 2 * np.pi * (pose < 0)
            robot_index = int(np.floor(pose / del_theta) )

            theta = min(abs(T - robot_index) * del_theta, 2 * np.pi - abs(T - robot_index) * del_theta)
            a = int(2 * T - robot_index)

            if theta == 2 * np.pi - abs(T - robot_index) * del_theta:
                a =int( 2 * T - robot_index)
                if a >= N:
                    a = a - N
                if a < 0:
                    a = a + N
                end_idx = int(min(a, robot_index))
                start_idx = int(max(a, robot_index))

                for i in range(start_idx, N):
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            print("151")
                            x_ref = np.real(x_ref)

                for i in range(0, end_idx):
                    if ranges[i] < x_ref * (np.cos(theta - (i + N - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i + N - start_idx) * del_theta) +
                                            (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            print("164")
                            x_ref = np.real(x_ref)

            elif theta == abs(T - robot_index) * del_theta:
                a = int(2 * T - robot_index)
                if a < 0 or a >= N:
                    if a > N:
                        a = a - N
                    if a < 0:
                        a = a + N

                    start_idx = int(max(a, robot_index))
                    end_idx = int(min(a, robot_index))
                    for i in range(start_idx, N):
                        if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                            x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                            if np.imag(x_ref) != 0:
                                print("187")
                                x_ref = np.real(x_ref)

                    for i in range(0, end_idx):
                        if ranges[i] < x_ref * (np.cos(theta - (i + N - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5):
                            x_ref = ranges[i] / (np.cos(theta - (i + N - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i + N - start_idx) * del_theta))**2)**0.5)
                            if np.imag(x_ref) != 0:
                                print("199")
                                x_ref = np.real(x_ref)

                elif abs(T - robot_index) == abs(T - a):
                    start_idx = int(min(a, robot_index))
                    end_idx = int(max(a, robot_index))
                    for i in range(start_idx, end_idx):
                        if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) +
                                                (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                            x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) +
                                                    (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                            if np.imag(x_ref) != 0:
                                print("212")
                                x_ref = np.real(x_ref)

        return x_ref, I

                                                                        
    def ipc_navigate(self, target_goal):
        self.transformedPose = np.array([self.pose[0] + self.L * math.cos(self.pose[2]),
                                         self.pose[1] + self.L * math.sin(self.pose[2]),
                                         self.angle_normalizer(self.pose[2] + self.pose[3]),
                                         self.angle_normalizer(self.pose[2])])        
    
        R_max = np.zeros(len(self.ranges))
        R_opt = np.zeros(len(self.ranges))
        D_check = np.zeros(len(self.ranges))
        for T in range(len(self.ranges)):
            R_max[T], _ = self.sensor_data_process(self.ranges, T, self.transformedPose[2])
            R_opt[T] = self.R_optimizer(self.target_goal[0], self.target_goal[1], self.transformedPose[0], self.transformedPose[1], T, R_max[T],
                                   self.angles, self.ranges)
            D_check[T] = self.euclidean_distance(target_goal[0], target_goal[1],
                                            self.transformedPose[0] + R_opt[T] * np.cos(self.angles[T]),
                                            self.transformedPose[1] + R_opt[T] * np.sin(self.angles[T]))

        i_best = np.argmin(D_check)
        theta = abs(self.transformedPose[3] - self.angles[i_best])
        way_x_opt = self.pose[0] + R_opt[i_best] * np.cos(self.angles[i_best])
        way_y_opt = self.pose[1] + R_opt[i_best] * np.sin(self.angles[i_best])
        self.active_waypoint = np.array([way_x_opt, way_y_opt])
        self.transformedPose = np.array([self.pose[0] + self.L * math.cos(self.pose[2]),
                                         self.pose[1] + self.L * math.sin(self.pose[2]),
                                         self.angle_normalizer(self.pose[2] + self.pose[3]),
                                         self.angle_normalizer(self.pose[2])])
        self.radius_icecone = self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                                      self.transformedPose[0], self.transformedPose[1]) * abs(
            np.sin(theta))

        start_point_x = self.transformedPose[0] + \
                        self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                                self.transformedPose[0], self.transformedPose[1]) * \
                        np.cos(theta) * np.cos(self.transformedPose[2])
        start_point_y = self.transformedPose[1] + \
                        self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                                self.transformedPose[0], self.transformedPose[1]) * \
                        np.cos(theta) * np.sin(self.transformedPose[2])

        end_point_x = self.transformedPose[0] + \
                      self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                              self.transformedPose[0], self.transformedPose[1]) * \
                      np.cos(theta) * np.cos(2 * self.angles[i_best] - self.transformedPose[2])
        end_point_y = self.transformedPose[1] + \
                      self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                              self.transformedPose[0], self.transformedPose[1]) * \
                      np.cos(theta) * np.sin(2 * self.angles[i_best] - self.transformedPose[2])

        self.start_point = np.array([start_point_x, start_point_y])
        self.end_point = np.array([end_point_x, end_point_y]) 
        R = np.linalg.norm(self.active_waypoint - self.transformedPose[:2])      
        rel_bearing = self.transformedPose[2] - np.arctan2(self.transformedPose[1] - way_y_opt, self.transformedPose[0] - way_x_opt)

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

        w_1 = -K_1 * np.tanh(R) * np.sign(np.cos(rel_bearing))
        w_2 = -K_2 * np.sign(sigma) * abs(sigma)**0.5 + (w_1 / R) * np.sin(rel_bearing)

        self.transformedPose += np.array([[math.cos(self.transformedPose[2]), 0],
                                          [math.sin(self.transformedPose[2]), 0],
                                          [0, 1],
                                          [np.sin(self.angle_normalizer(self.transformedPose[2] -
                                                                        self.transformedPose[3])) / self.L, 0]]) @ \
                               np.array([w_1, w_2])*self.del_T 

        self.transformedPose[2] = self.angle_normalizer(self.transformedPose[2])
        self.transformedPose[3] = self.angle_normalizer(self.transformedPose[3])

        V = np.linalg.solve(np.array([[1, 0],
                                      [np.sin(self.angle_normalizer(self.transformedPose[2] -
                                                                     self.transformedPose[3])) / self.L, 1]]),
                            np.array([w_1, w_2]))
        omega = V[1]
        #self.steer_angle += omega*self.del_T
        #V[0] = V[0]*np.cos(self.steer_angle)
        #print("steer angle",self.steer_angle)
        #self.controls = np.array([V[0], self.steer_angle])
        self.controls = np.array([10, 0.2])
        return self.controls   
    
if __name__ == '__main__':    
    try:
        time.sleep(5)
        BicycleMobileRobot()
    except rospy.ROSInterruptException:
        pass
  






