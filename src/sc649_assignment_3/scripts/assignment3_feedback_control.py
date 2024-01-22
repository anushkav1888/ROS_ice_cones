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
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose
import tf

class BicycleMobileRobot:
    def __init__(self,robot_name=None):
        rospy.init_node('main_controller', anonymous=True)
        self.L = 0.40
        #self.name = robot_name
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
        self.target_goal = [50,0 ,0]
        self.linear_velocity_x = 0
        self.linear_velocity_y = 0
        self.angular_velocity_z = 0
        self.steer_angle = 0
        self.min_angle = 0
        self.max_angle = 0
        self.range_cal = []

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.lidar_callback, queue_size=10)
        rospy.Subscriber('/pf/pose/odom', Odometry, self.RobotPose)
        self.pub = rospy.Publisher('/racecar/ackermann_cmd_mux/output', AckermannDriveStamped, queue_size=10)
        self.waypoint_publisher = rospy.Publisher('/waypoints', MarkerArray, queue_size=10)
        self.velocity_msg = AckermannDriveStamped()
        self.rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            start_time = time.time()

            if len(self.pose) == 0:
                continue

            if len(self.ranges) == 0:
                continue

            [self.velocity_msg.drive.speed, self.velocity_msg.drive.steering_angle] = self.ipc_navigate(self.target_goal)
            #turtlebot3_0.ipc_navigate([10,2])
            #turtlebot3_1.ipc_navigate([6.25,2])
            #turtlebot3_2.ipc_navigate([-10.0,2])
            #turtlebot3_3.ipc_navigate([-1.0,20.75])
            #turtlebot3_4.ipc_navigate([-1.0,10.25])
            #[self.velocity_msg.drive.speed, self.velocity_msg.drive.steering_angle] = self.ipc_navigate(
             #self.target_goal)
            loop_duration = time.time() - start_time
            #print(self.speed,self.velocity_msg.drive.speed, loop_duration)
            self.velocity_msg.drive.speed = self.velocity_msg.drive.speed*2
            self.pub.publish(self.velocity_msg)
            waypoints = self.generate_circle_markers(self.range_cal,self.active_waypoint, self.radius_icecone, self.start_point, self.end_point, self.pose, self.target_goal)  # Implement your waypoint generation logic
            marker_array = MarkerArray(markers=waypoints)
            self.waypoint_publisher.publish(marker_array)
                # Create an instance of the TurtleBot3 class

            #print(time.time())
            # Print the loop duration
            #print(f"Loop Time: {loop_duration:.6f} seconds")
            #start_time = time.time()
            self.rate.sleep()

    def generate_circle_markers(self, range_cal,center, radius, start_point, end_point, pose, target_goal,num_points=100):
        markers = []

        # Generate circle markers
        for i in range(num_points):
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.05  # Point size
            marker.scale.y = 0.05
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            angle = 2.0 * math.pi * i / num_points
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            z = 0

            point = Point()
            point.x = x
            point.y = y
            point.z = z

            marker.points.append(point)
            markers.append(marker)
        
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "base_link"
        arrow_marker.id = len(markers)  # Ensure a unique ID
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        # Assuming robot_pose is a Pose object with position and orientation
        robot_pose = Pose()
        robot_pose.position.x = pose[0]  # Replace with actual robot position
        robot_pose.position.y = pose[1]
        robot_pose.position.z = 0.0

        # Calculate quaternion for desired incline angle (e.g., 45 degrees)
        
        quaternion = [1,0,0,0]
        incline_angle = self.pose[2]*180/np.pi  # Replace with your desired angle in degrees
        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, math.radians(incline_angle), axes='sxyz')

        robot_pose.orientation.x = quaternion[0]
        robot_pose.orientation.y = quaternion[1]
        robot_pose.orientation.z = quaternion[2]
        robot_pose.orientation.w = quaternion[3]

        #if robot_pose.orientation.x == 0:
         #    robot_pose.orientation.x = 0.001
        arrow_marker.pose = robot_pose
        arrow_marker.scale.x = 1.0  # Arrow shaft diameter
        arrow_marker.scale.y = 0.1  # Arrow head diameter
        arrow_marker.scale.z = 0.1  # Arrow head length
        arrow_marker.color.a = 1.0
        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0

        markers.append(arrow_marker)
        
        
        # Draw a line joining two pints
        line_marker = Marker()
        line_marker.header.frame_id = "base_link"
        line_marker.id = len(markers)  # Ensure a unique ID
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD

        # Assuming point1 and point2 are Point objects
        point1 = Point(x=start_point[0], y=start_point[1], z=0.0)  # Replace with actual point coordinates
        point2 = Point(x=pose[0], y=pose[1], z=0.0)

        line_marker.points.append(point1)
        line_marker.points.append(point2)
        line_marker.scale.x = 0.05  # Line width
        line_marker.color.a = 1.0
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0

        markers.append(line_marker)
        # Draw another line between two points
        line_marker_2 = Marker()
        line_marker_2.header.frame_id = "base_link"
        line_marker_2.id = len(markers)  # Ensure a unique ID
        line_marker_2.type = Marker.LINE_STRIP
        line_marker_2.action = Marker.ADD

        # Assuming point3 and point4 are Point objects
        point3 = Point(x=end_point[0], y=end_point[1], z=0.0)   # Replace with actual point coordinates
        point4 = Point(x=pose[0], y=pose[1], z=0.0)

        line_marker_2.points.append(point3)
        line_marker_2.points.append(point4)
        line_marker_2.scale.x = 0.05  # Line width
        line_marker_2.color.a = 1.0
        line_marker_2.color.r = 0.0
        line_marker_2.color.g = 1.0
        line_marker_2.color.b = 0.0

        markers.append(line_marker_2)
        """
        angle_Arr = np.array(self.angles) + self.pose[2]
        for i, angle in enumerate(angle_Arr):
            # Assuming range_cal is an array of range readings
            range_value = range_cal[i]

            # Calculate the point based on range and angle
            x = self.transformedPose[0] + range_value * np.cos(angle)
            y = self.transformedPose[1] + range_value * np.sin(angle) 
            z = 0.0
            #print(x,y,z)
            # Create a Point message for the marker
            point = Point()
            point.x = x
            point.y = y
            point.z = z

            # Create a Marker for each point
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.id = i
            marker.type = Marker.POINTS  # You can use SPHERE or POINTS based on your preference
            marker.action = Marker.ADD
            marker.pose.position = point
            marker.scale.x = 2  # Adjust the size of the marker
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.points.append(point)

            # Append the marker to the MarkerArray
            markers.append(marker)
        """
        # Add Sphere Marker
        sphere_marker = Marker()
        sphere_marker.header.frame_id = "base_link"
        sphere_marker.id = len(markers)  # Ensure a unique ID
        sphere_marker.type = Marker.SPHERE
        sphere_marker.action = Marker.ADD
        sphere_marker.pose.position.x = self.target_goal[0]  # Use the position of the point as the position of the sphere
        sphere_marker.pose.position.y = self.target_goal[1]  # Use the position of the point as the position of the sphere
        sphere_marker.pose.position.z = self.target_goal[2]  # Use the position of the point as the position of the sphere
        sphere_marker.scale.x = 0.15  # Sphere diameter
        sphere_marker.scale.y = 0.15
        sphere_marker.scale.z = 0.15
        sphere_marker.color.a = 1.0
        sphere_marker.color.r = 0.0
        sphere_marker.color.g = 0.0
        sphere_marker.color.b = 1.0

        markers.append(sphere_marker)

        return markers
    
    def lidar_callback(self, msg):
        self.ranges = msg.ranges
        self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        self.min_angle = msg.angle_min
        self.max_angle = msg.angle_max
        if len(self.ranges) >271:
            self.ranges = [self.ranges[i] for i in range(len(self.ranges)) if i % 4 == 0]
        if len(self.angles) > 271:    
            self.angles = [self.angles[i] for i in range(len(self.angles)) if i % 4 == 0]
        for i in range(130,141):
            if self.ranges[i]  > 0.3:
                self.ranges[i] = self.ranges[i] - 0.5
                #print("range enc")
            else:
                self.ranges[i] = 0.01      
                #print("no ran")
        #print(180/np.pi*msg.angle_min,180/np.pi*msg.angle_max)
        #print(len(self.ranges))

    def RobotPose(self, data):
        yaw = euler_from_quaternion([data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                                           data.pose.pose.orientation.z, data.pose.pose.orientation.w])[2]
        self.pose = [data.pose.pose.position.x, data.pose.pose.position.y,yaw,self.steer_angle]
        self.linear_velocity_x = data.twist.twist.linear.x
        self.linear_velocity_y = data.twist.twist.linear.y
        self.angular_velocity_z = data.twist.twist.angular.z
        self.speed = np.sqrt((self.linear_velocity_x ) ** 2 + (self.linear_velocity_y) ** 2)

    def euclidean_distance(self, x1, y1, x2, y2):
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def angle_normalizer(self, angle):
        if angle < -math.pi:
            out = angle + 2 * math.pi
        elif angle > math.pi:
            out = angle - 2 * math.pi
        else:
            out = angle
        return out
    
    def R_optimizer(self, int_goal_x, int_goal_y, Robot_x, Robot_y, T, R_max, angles, ranges, pose):
        angle_diff = angles[T+135]+pose - np.arctan2(int_goal_y, int_goal_x)
        R_opt = 0
        if T <= 67 and T >= -67:
            if np.cos(angle_diff) > 0:
                distance_to_goal = self.euclidean_distance(int_goal_x, int_goal_y, Robot_x, Robot_y)
                if distance_to_goal * np.cos(angle_diff) >= R_max:
                    R_opt = R_max
                    #print("way1", R_max)
                else:
                    R_opt = distance_to_goal * np.cos(angle_diff)
                if np.cos(angle_diff) <= 0:
                     print("issue")    
            else:
                R_opt = 0
                #print("Way2")
            #print("opt", R_opt, T)    
        return R_opt


    def sensor_data_process(self,ranges, T, pose,angles):
        #lim_left = np.argmin(np.abs(np.array(angles) + np.pi*3/8)) #left index for ice cone to be lidar range
        #lim_right = np.argmin(np.abs(np.array(angles) - np.pi*3/8)) 
        robot_index = int(np.where(np.array(angles)==0)[0])
        #print(robot_index,"rob")
        N = len(ranges)
        del_theta = 3/2*np.pi/N
        theta = np.abs(T)*del_theta

        I = 0
        #print(T,"tt")
        #print(robot_index+T)
        x_ref = ranges[robot_index+T]
        #print(lim_right-135,"lim")
        if T > 67 or T < -67:
            x_ref = 0    
        elif T <= 67 and T >= -67:
            if T > 0:
                    for i in np.array(range(0, 2*T)):  # moving till N and then to end_index which is in the first quadrant
                        #print(2*T)
                        if ranges[i+robot_index] < x_ref * (np.cos(theta - i * del_theta) +
                                                np.sqrt((-np.cos(theta)**2 + (np.cos(theta - i * del_theta))**2))):
                            x_ref = ranges[i+robot_index] / (np.cos(theta - i * del_theta) +
                                                np.sqrt((-np.cos(theta)**2 + (np.cos(theta - i * del_theta))**2)))
                            if np.imag(x_ref) != 0:
                                 print(217)
            if T < 0:
                    for i in np.array(range(2*T,1)):  # moving till N and then to end_index which is in the first quadrant
                                        if ranges[i+robot_index] < x_ref * (np.cos(theta + i * del_theta) +
                                                                np.sqrt((-np.cos(theta)**2 + (np.cos(theta + i * del_theta))**2))):
                                            x_ref = ranges[i+robot_index] / (np.cos(theta + i * del_theta) +
                                                                np.sqrt((-np.cos(theta)**2 + (np.cos(theta + i * del_theta))**2)))
                                            if np.imag(x_ref) != 0:
                                                print(227)                
        return x_ref, I
                          
                                                             
    def ipc_navigate(self, target_goal):
        self.transformedPose = np.array([self.pose[0] + self.L * math.cos(self.pose[2]),
                                         self.pose[1] + self.L * math.sin(self.pose[2]),
                                         self.angle_normalizer(self.pose[2] + self.pose[3]),
                                         self.angle_normalizer(self.pose[2])])        
    
        
        if len(self.ranges )== 1081:
            self.ranges = [self.ranges[i] for i in range(len(self.ranges)) if i % 4 == 0]
            #print("rec ranges")

        if len(self.angles) == 1081:
            #print("len angles", len(self.angles),"ipc")
            self.angles = [self.angles[i] for i in range(len(self.angles)) if i % 4 == 0]
            

        R_max = np.zeros(len(self.ranges))
        R_opt = np.zeros(len(self.ranges))
        D_check = np.zeros(len(self.ranges))
        for T in np.array(range(-135,136)):     
            if len(self.ranges )== 1081:
                self.ranges = [self.ranges[i] for i in range(len(self.ranges)) if i % 4 == 0]
                #print("rec ranges")

            if len(self.angles) == 1081:
                #print("len angles", len(self.angles),"ipc")
                self.angles = [self.angles[i] for i in range(len(self.angles)) if i % 4 == 0]

            R_max[T+135], _ = self.sensor_data_process(self.ranges, T, self.transformedPose[2],self.angles)
            R_opt[T+135] = self.R_optimizer(target_goal[0], target_goal[1], self.transformedPose[0], self.transformedPose[1], T, R_max[T+135],
                                   self.angles, self.ranges,self.pose[2])
            D_check[T+135] = self.euclidean_distance(target_goal[0], target_goal[1],
                                            self.transformedPose[0] + R_opt[T+135] * np.cos(self.angles[T+135]+self.pose[2]),
                                            self.transformedPose[1] + R_opt[T+135] * np.sin(self.angles[T+135]+self.pose[2]))
            #if T > -67 and T < 67 :
                
                #print(T,R_opt[T+135] * np.cos(self.angles[T]+self.pose[2]))
            
            #print(T+135,R_max[T+135],R_opt[T+135],D_check[T+135])
            #print(D_check[T+135],self.transformedPose[0] + R_opt[T+135] * np.cos(self.angles[T]+self.pose[2]))
            if len(D_check) > 271:
                break
        self.range_cal = R_max    
        #print(R_max,"rmax", R_opt)
        #print(D_check[68:203])    
        i_best = 67+np.argmin(D_check[68:203])
        #if i_best <67 or i_best > 202:
        #    print("isssue")
        #if R_opt[i_best] == 0:
        #    print(i_best , "is 0")
             #print(R_max[68:203])
        #print(D_check[68:203])     
        #print("ii",i_best)
        #print("lend",len(D_check))     
        #theta = min(abs(self.transformedPose[3] - self.angles[i_best]), 2*np.pi-abs(self.transformedPose[3] - self.angles[i_best]))
        
        way_x_opt = self.transformedPose[0] + R_opt[i_best] * np.cos(self.angles[i_best]+self.pose[2])
        way_y_opt = self.transformedPose[1] + R_opt[i_best] * np.sin(self.angles[i_best]+self.pose[2])
        #print(way_x_opt)
        #theta = self.transformedPose[2] - np.arctan2(self.transformedPose[1] - way_y_opt, self.transformedPose[0] - way_x_opt)
        rel_bearing = self.transformedPose[2] - np.arctan2(self.transformedPose[1] - way_y_opt, self.transformedPose[0] - way_x_opt)
        #print("rel", rel_bearing)

        rel_bearing = self.angle_normalizer(rel_bearing)

        if rel_bearing >= np.pi / 2:
            sigma = rel_bearing - np.pi
        elif rel_bearing < -np.pi / 2:
            sigma = rel_bearing + np.pi
        else:
            sigma = rel_bearing
        #print(rel_bearing)    
        self.active_waypoint = np.array([way_x_opt, way_y_opt])
        self.transformedPose = np.array([self.pose[0] + self.L * math.cos(self.pose[2]),
                                         self.pose[1] + self.L * math.sin(self.pose[2]),
                                         self.angle_normalizer(self.pose[2] + self.pose[3]),
                                         self.angle_normalizer(self.pose[2])])
        self.radius_icecone = self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                                      self.transformedPose[0], self.transformedPose[1]) * abs(np.sin(rel_bearing))
            
        #print(self.radius_icecone, "radii")

        start_point_x = self.transformedPose[0] - self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],self.transformedPose[0], self.transformedPose[1])*np.cos(rel_bearing) * np.cos(self.transformedPose[2])
                                                                        
        start_point_y = self.transformedPose[1] - self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],self.transformedPose[0], self.transformedPose[1]) * np.cos(rel_bearing) * np.sin(self.transformedPose[2])

        end_point_x = self.transformedPose[0] - self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],self.transformedPose[0], self.transformedPose[1]) * \
                      np.cos(rel_bearing) * np.cos(2 * (self.angles[i_best]+self.pose[2]) - self.transformedPose[2])
        end_point_y = self.transformedPose[1] - \
                      self.euclidean_distance(self.active_waypoint[0], self.active_waypoint[1],
                                              self.transformedPose[0], self.transformedPose[1]) * \
                      np.cos(rel_bearing) * np.sin(2 * (self.angles[i_best]+self.pose[2]) - self.transformedPose[2])

        self.start_point = np.array([start_point_x, start_point_y])
        self.end_point = np.array([end_point_x, end_point_y]) 
        R = self.euclidean_distance(self.active_waypoint[0] ,self.active_waypoint[1], self.transformedPose[0],self.transformedPose[1])   
        #print(R,"r")   
        K_1 = 0.1
        K_2 = 2
        if R == 0:
             R = 0.001
        w_1 = -K_1 * np.tanh(R) * np.sign(np.cos(rel_bearing))
        w_2 = -K_2 * np.sign(sigma) * np.sqrt(np.abs(sigma))+ (w_1 / R) * np.sin(rel_bearing)
        #print(w_1,"w1")
        #print(sigma, "sig")
        #print(w_2,"w2")

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
        self.steer_angle += omega*1/1000
        
        V[0] = V[0]*np.cos(self.steer_angle)
        #print("steer angle",self.steer_angle)
        #print(V[0])
        self.controls = np.array([V[0], self.steer_angle])
        if self.euclidean_distance(self.pose[0], self.pose[1], target_goal[0], target_goal[1]) < 0.1:
             print("reached goal")
             self.controls = np.array([0,0])
        #self.controls = np.array([5, np.pi/6])
        return self.controls 
      
class TurtleBot3(BicycleMobileRobot):
    def __init__(self, robot_name):
        super(TurtleBot3, self).__init__(robot_name)

    def update(self):
        # Example control logic:
        # Stop if the robot is close to an obstacle
        if self.odometry.pose.pose.position.x < 0.1:
            self.publish_velocity(0.0, 0.0)
        else:
            # Move forward at a constant linear velocity
            self.publish_velocity(0.2, 0.0)

if __name__ == '__main__':    
    try:
        time.sleep(5)
        BicycleMobileRobot()
        #turtlebot3_0 = BicycleMobileRobot('tb3_0')
        #turtlebot3_1 = BicycleMobileRobot('tb3_1')
        #turtlebot3_2 = BicycleMobileRobot('tb3_2')
        #turtlebot3_3 = BicycleMobileRobot('tb3_3')
        #turtlebot3_4 = BicycleMobileRobot('tb3_4')
    except rospy.ROSInterruptException:
        pass
  






