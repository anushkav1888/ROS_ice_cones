import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
import math
import numpy as np
from scipy.optimize import minimize

class BicycleMobileRobot(Node):
    def __init__(self, node_name='bicycle_mobile_robot'):
        super().__init__(node_name)
        self.pose = np.zeros(4)
        self.transformed_pose = np.zeros(4)
        self.controls = np.zeros(2)
        self.transformed_controls = np.zeros(2)
        self.L = 0
        self.del_T = 0
        self.active_waypoint = np.zeros(2)
        self.ref_map = None
        self.lidar = None
        self.subsampling_index = 0
        self.radius_icecone = 0
        self.start_point = np.zeros(2)
        self.end_point = np.zeros(2)
        self.max_range = None
        self.lidar = self.create_subscription(
            Range,
            '/lidar_topic',
            self.lidar_callback,
            10
        )
    def lidar_callback(self, msg):
        pass

    def update(self, control_inputs):
        transformation_matrix = np.array([
            [np.cos(self.pose[2]), 0],
            [np.sin(self.pose[2]), 0],
            [0, 1]
        ])
        self.pose += np.dot(transformation_matrix, control_inputs) * self.del_T

    def angle_normalizer(self, angle):
        if angle < -np.pi:
            out = angle + 2 * np.pi
        elif angle > np.pi:
            out = angle - 2 * np.pi
        else:
            out = angle
        return out

    def tau(self, s):
        a = 250  
        b = 130  
        x = a * np.cos(2 * np.pi * s) + 400  # x-coordinate of the ellipse
        y = b * np.sin(2 * np.pi * s) + 400  # y-coordinate of the ellipse
        return np.array([x, y])

    def sensor_data_process(self, ranges, T, pose):
        I = 0
        x_ref = ranges[T]
        N = len(ranges)
        del_theta = 2 * np.pi / N

        pose = np.mod(pose, 2 * np.pi)
        robot_index = round(pose / del_theta) + 1

        if robot_index > N:
            robot_index = 1

        theta = min(abs(T - robot_index) * del_theta, 2 * np.pi - abs(T - robot_index) * del_theta)

        if theta >= np.pi / 2:
            if pose <= np.pi:
                robot_index = round((pose + np.pi) / del_theta) + 1
                if robot_index > N:
                    robot_index = 1

            if pose > np.pi:
                robot_index = round((pose - np.pi) / del_theta) + 1

            theta = min(abs(T - robot_index) * del_theta, 2 * np.pi - abs(T - robot_index) * del_theta)

            if theta == abs(T - robot_index) * del_theta:
                a = 2 * T - robot_index
                if a > N:
                    a = a - N

                end_idx = min(a, robot_index)
                start_idx = max(a, robot_index)

                for i in range(start_idx, N):
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            a = 4
                for i in range(0, end_idx):
                    if ranges[i] < x_ref * (np.cos(theta - (i - 1) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - 1) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i - 1) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - 1) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            a = 5
            elif a < 1:
                a = a + N
                end_idx = min(a, robot_index)
                start_idx = max(a, robot_index)

                for i in range(start_idx, N):
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**2)**0.5)
                        if np.imag(x_ref) != 0:
                            a = 6
                for i in range(0, end_idx):
                    if ranges[i] < x_ref * (np.cos(theta - (i - 1) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - 1) * del_theta))**0.5)):
                        x_ref = ranges[i] / (np.cos(theta - (i - 1) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - 1) * del_theta))**0.5))
                        if np.imag(x_ref) != 0:
                            a = 7
            else:
                start_idx = min(a, robot_index)
                end_idx = max(a, robot_index)

                for i in range(start_idx, end_idx):
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx)* del_theta))**0.5)):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**0.5))
  
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
                    if ranges[i] < x_ref * (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**0.5)):
                        x_ref = ranges[i] / (np.cos(theta - (i - start_idx) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - start_idx) * del_theta))**0.5))
                        if np.imag(x_ref) != 0:
                            a = 2

                for i in range(0, end_idx):
                    if ranges[i] < x_ref * (np.cos(theta - (i - 1) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - 1) * del_theta))**0.5)):
                        x_ref = ranges[i] / (np.cos(theta - (i - 1) * del_theta) + (-np.cos(theta)**2 + (np.cos(theta - (i - 1) * del_theta))**0.5))
                        if np.imag(x_ref) != 0:
                            a = 3
            return x_ref, I

    def distance_to_P(self, s, P):
        point_on_curve = self.tau(s)
        distance = np.linalg.norm(point_on_curve - P)**2
        return distance

    def furthest_along_track_computer(self, robot_fx, robot_fy, R_max, angles):
        s_furthest = 0
        optimal_P = np.array([robot_fx, robot_fy])
        can_s_star = np.zeros(len(R_max))
        can_optimal_P = np.zeros((2, len(R_max)))

        for j in range(len(R_max)):
            P = np.array([robot_fx + R_max[j] * np.cos(angles[j]), robot_fy + R_max[j] * np.sin(angles[j])])

            # Initial guess for 's*'.
            initial_guess_s = 1
            # Minimize the distance function using minimize.
            result = minimize(lambda s: self.distance_to_P(s, P), initial_guess_s, bounds=[(0, 1)])
            s_star = result.x[0]
            # Evaluate tau(s*) to get the 2D point on the ellipse.
            # optimal_point_on_curve = tau(s_star)
            can_s_star[j] = s_star
            can_optimal_P[:, j] = P

        index = np.argmax(can_s_star)
        s_furthest = can_s_star[index]
        optimal_P = can_optimal_P[:, index]

        return s_furthest, optimal_P, index

    def euclidean_distance(self, gap_f_x, gap_f_y, can_gap_s_x, can_gap_s_y):
        d = ((gap_f_x - can_gap_s_x)**2 + (gap_f_y - can_gap_s_y)**2)**0.5
        return d

    def ipc_navigate(self, other_robots):
        constructed_range_array = self.ranges

        # Iterating through each angle index for figuring out the waypoint for navigation
        for T in range(len(self.ranges)):
            R_max, I = self.sensor_data_process(constructed_range_array, T, self.transformed_pose[2])

        # Computing farthest along track
        s_furthest, optimal_P, i_best = self.furthest_along_track_computer(self.transformed_pose[0],
                                                                        self.transformed_pose[1],
                                                                        R_max, self.angles)
        self.active_waypoint = optimal_P
        self.max_range = R_max

        self.transformed_pose[2] = self.transformed_pose[2] % (2 * np.pi)
        self.angles[i_best] = self.angles[i_best] % (2 * np.pi)

        theta = np.abs(self.transformed_pose[2] - self.angles[i_best])
        the = theta * 180 / np.pi
        pang = self.angles[i_best] * 180 / np.pi
        rob = self.transformed_pose[2] * 180 / np.pi

        self.radius_icecone = self.euclidean_distance(optimal_P[0], optimal_P[1],
                                                    self.transformed_pose[0], self.transformed_pose[1]) * np.abs(np.sin(theta))

        start_point_x = self.transformed_pose[0] + self.euclidean_distance(optimal_P[0], optimal_P[1],
                                                                        self.transformed_pose[0], self.transformed_pose[1]) \
                        * np.cos(theta) * np.cos(self.transformed_pose[2])
        start_point_y = self.transformed_pose[1] + self.euclidean_distance(optimal_P[0], optimal_P[1],
                                                                        self.transformed_pose[0], self.transformed_pose[1]) \
                        * np.cos(theta) * np.sin(self.transformed_pose[2])

        end_point_x = self.transformed_pose[0] + self.euclidean_distance(optimal_P[0], optimal_P[1],
                                                                    self.transformed_pose[0], self.transformed_pose[1]) \
                        * np.cos(theta) * np.cos(2 * self.angles[i_best] - self.transformed_pose[2])
        end_point_y = self.transformed_pose[1] + self.euclidean_distance(optimal_P[0], optimal_P[1],
                                                                    self.transformed_pose[0], self.transformed_pose[1]) \
                        * np.cos(theta) * np.sin(2 * self.angles[i_best] - self.transformed_pose[2])

        self.start_point = np.array([start_point_x, start_point_y])
        self.end_point = np.array([end_point_x, end_point_y])

        # Controller Implementation
        R = np.linalg.norm(optimal_P - self.transformed_pose[0:2])
        rel_bearing = self.transformed_pose[2] - np.arctan2(self.transformed_pose[1] - self.active_waypoint[1],
                                                            self.transformed_pose[0] - self.active_waypoint[0])
        rel_bearing = self.angle_normalizer(rel_bearing)

        if rel_bearing >= np.pi / 2:
            sigma = rel_bearing - np.pi
        elif rel_bearing <= -np.pi / 2:
            sigma = rel_bearing + np.pi
        else:
            sigma = rel_bearing

        K_1 = 10
        K_2 = 5

        w_1 = -K_1 * np.tanh(R) * np.sign(np.cos(rel_bearing))
        w_2 = -K_2 * np.sign(sigma) * np.abs(sigma)**(0.5) + (w_1 / R) * np.sin(rel_bearing)

        self.transformed_pose = self.transformed_pose + np.array([
            [np.cos(self.transformed_pose[2]), 0],
            [np.sin(self.transformed_pose[2]), 0],
            [0, 1],
            [np.sin(self.angle_normalizer(self.transformed_pose[2] - self.transformed_pose[3])) / self.L, 0]
        ]) @ np.array([w_1, w_2]) * self.del_T

        self.transformed_pose[2] = self.angle_normalizer(self.transformed_pose[2])
        self.transformed_pose[3] = self.angle_normalizer(self.transformed_pose[3])

        V = np.linalg.inv([
            [1, 0],
            [np.sin(self.angle_normalizer(self.transformed_pose[2] - self.transformed_pose[3])) / self.L, 1]
        ]) @ np.array([w_1, w_2])

        self.pose = self.pose + np.array([
            [np.cos(self.pose[2]) * np.cos(self.pose[3]), 0],
            [np.sin(self.pose[2]) * np.cos(self.pose[3]), 0],
            [np.sin(self.pose[3]) / self.L, 0],
            [0, 1]
        ]) @ np.array([V[0], V[1]]) * self.del_T

        self.pose[2] = self.angle_normalizer(self.pose[2])
        self.pose[3] = self.angle_normalizer(self.pose[3])
        self.controls = V  # Initial velocity

