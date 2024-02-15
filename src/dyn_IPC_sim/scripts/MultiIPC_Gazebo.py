#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from detect_gaps import range_smoother,R_max_finder,waypoint_finder_CODED
from sensor_msgs.msg import LaserScan
import time

class RobotBase:
    def __init__(self, robot_name):
        self.name = robot_name
        self.velocity = Twist()
        self.odometry = Odometry()
        self.scan = LaserScan()
        self.pose = []
    
        self.odom_subscriber = rospy.Subscriber(f'/{self.name}/odom', Odometry, self.odometry_callback) #Subscriber to listen to the Odometry topic
        #self.vicon_subscriber = rospy.Subscriber(f'/vicon/{self.name}/{self.name}', TransformStamped, self.callback_vicon)
        #self.vicon_subscriber = rospy.Subscriber('/vicon/tb3_4/tb3_4', TransformStamped, self.callback_vicon)
        self.LiDAR_subscriber = rospy.Subscriber(f'/{self.name}/scan', LaserScan, self.callback_LiDAR) #Subscriber to listen to onboard LiDAR
        self.velocity_publisher = rospy.Publisher(f'/{self.name}/cmd_vel', Twist, queue_size=10) #Publisher to send velocity commands
        # with open(f"~/Music/{self.name}_data_pose_plan.txt", "w") as file_pose_plan:

    def odometry_callback(self, data):
        # Store the received odometry data
        self.odometry = data
        self.pose = [data.pose.pose.position.x,data.pose.pose.position.y,euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]]


    def publish_velocity(self, linear_x, angular_z):
        # Create a Twist message with the desired linear and angular velocities
        cmd_vel = Twist()
        cmd_vel.linear.x = linear_x
        cmd_vel.angular.z = angular_z
        
        # Publish the velocity command
        self.velocity_publisher.publish(cmd_vel)

    def update(self):
        # Implement your robot control logic here
        # For example, you can use the odometry data to make decisions
        # and then use self.publish_velocity() to set the robot's velocity
        pass

    ## quaternion to euler
    def quat2euler(self,x,y,z,w):
        quat = [x,y,z,w]
        return euler_from_quaternion(quat)
    ########################

    ## Odometry callback
    #def callback_vicon(self,data):
        #global pose
        #print('Pose obtained from Vicon')
        #x  = data.transform.rotation.x
        #y  = data.transform.rotation.y
        #z  = data.transform.rotation.z
        #w  = data.transform.rotation.w
        #self.pose = [data.transform.translation.x, data.transform.translation.y, self.quat2euler(x,y,z,w)[2]]
    ########################

    #def RobotPose(data):
        #global pose
        #pose = [data.pose.pose.position.x,data.pose.pose.position.y,euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]]

    ## LiDAR scan
    def callback_LiDAR(self,data):
        #global scan 
        self.scan = data
        #print('Scan obtained from LiDAR')
    
    ## Euclidean Distance
    def dist(self,p1, p2):
        return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)
    ########################

    def bearing(self, p1, p2):
        return ( math.atan2(p2[1]-p1[1],p2[0]-p1[0]) )
    ########################

    def signum(self,p1):
        if(p1>=0):
            return 1
        else:
            return -1

    def store_data_pose_plan(self,vector1,vector2):
        #data = f"{vector1[0]:.4f} {vector1[1]:.4f} {vector1[2]:.4f} {vector2[0]:.4f} {vector2[1]:.4f} {vector2[2]:.4f}\n"
        print("func1")
        with open(f"/home/veejay/Music/{self.name}_data_pose_plan.txt", "a+") as file_pose_plan:
            file_pose_plan.write("{}\t{}\t{}\t{}\t{}\t{}\n".format(vector1[0],vector1[1],vector1[2],vector2[0],vector2[1],vector2[2]))
        # file_pose_plan.close()

    def store_data_control_inputs(self,vector1):
        #data = f"{vector1[0][0]:.4f} {vector1[1][0]:.4f} {vector1[2][0]:.4f}\n"
        print("func2")
        with open(f"/home/veejay/Music/{self.name}_data_control.txt", "a+") as file_control_inputs:
            file_control_inputs.write("{}\t{}\n".format(vector1[0],vector1[1]))
        # file_control_inputs.close()


    def fetch_waypoint(self,current_Gap):

        R_max_set = R_max_finder(range_smoother(self.scan.ranges,self.scan.range_min,self.scan.range_max))
        [waypoint_distance,waypoint_index,selected_gap_distance] = waypoint_finder_CODED(current_Gap,R_max_set,range_smoother(self.scan.ranges,self.scan.range_min,self.scan.range_max),self.pose)
        #[waypoint_distance,waypoint_index,selected_gap_distance] = waypoint_finder_CODED(current_Gap,R_max_set,scan.ranges,pose)
        N_ranges = len(R_max_set)
        del_theta_measure = 2*np.pi/N_ranges


        way_x = self.pose[0] + waypoint_distance*math.cos((waypoint_index*del_theta_measure) + 1*self.pose[2])
        way_y = self.pose[1] + waypoint_distance*math.sin((waypoint_index*del_theta_measure) + 1*self.pose[2])

        way = [way_x,way_y]
    
        return way,selected_gap_distance
    
    ## Main Node
    def controller(self,Goal):

        #global pose,scan
    
        velocity_msg = Twist()
        rate = rospy.Rate(10)

        while(len(self.scan.ranges)==0):
            print('First scan not obtained yet')
            continue
        while(len(self.pose)==0):
            print('First pose not obtained yet')
            continue

        integral_action = 0
        total_plan_time = 0
        planning_instances = 0

        current_target_gap = Goal

        K_1 = 0.3
        K_2 = 0.5
        K_3 = 0

        Robot_position = [self.pose[0],self.pose[1]]
        

        start_plan_time = time.time()
        try:
            way,selected_gap_distance = self.fetch_waypoint(current_target_gap)
            end_plan_time = time.time()
            total_plan_time = total_plan_time + (end_plan_time-start_plan_time)

        #avg_waypoint_planning_duration = total_plan_time/planning_instances 
        
        #way = Gap_list[current_gap_index]
            R = self.dist(way,Robot_position)
            rel_bearing = self.pose[2] - self.bearing(way,Robot_position)
            print(way)
        #To bring the relative bearing values in the range of [pi,-pi)
            if(rel_bearing>np.pi):
                rel_bearing = rel_bearing - 2*np.pi
            #continue
        
            if(rel_bearing<-np.pi):
                rel_bearing = rel_bearing + 2*np.pi 
            #continue

            if(math.cos(rel_bearing)<0):
                S = rel_bearing - self.signum(rel_bearing)*np.pi
            else:
                S = rel_bearing

            integral_action = integral_action + (-K_3*self.signum(S)*0.1) #rospy.Rate is set at 10 Hz
            linear_vel = -K_1*math.tanh(R)*self.signum(math.cos(rel_bearing))
            velocity_msg.linear.x = linear_vel
        
            try:
                angular_vel = -K_2*(abs(S)**0.5)*self.signum(S) + integral_action + K_1*self.signum(math.cos(rel_bearing))*math.sin(rel_bearing)*math.tanh(R)/R
            except Exception as e:
                angular_vel = -K_2*(abs(S)**0.5)*self.signum(S) + integral_action + K_1*self.signum(math.cos(rel_bearing))*math.sin(rel_bearing)


            velocity_msg.angular.z = angular_vel
            
            if(self.dist(Goal,Robot_position)<0.1):
                velocity_msg.angular.z = 0
                velocity_msg.linear.x = 0
            else:
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel


            self.velocity_publisher.publish(velocity_msg)

            #try:
            self.store_data_pose_plan([self.pose[0],self.pose[1],self.pose[2]],[way[0],way[1],R])
            #except Exception as e:
            #print("error func1",e)
            self.store_data_control_inputs([linear_vel,angular_vel])

        except:
            pass
        
        #store_data_pose_plan([pose[0],pose[1],pose[2]],[way[0],way[1],R])
        #store_data_control_inputs([linear_vel,angular_vel])

        #rate.sleep()
########################

class TurtleBot3(RobotBase):
    def __init__(self, robot_name):
        super(TurtleBot3, self).__init__(robot_name)

    def update(self):
        # Example control logic:
        # Stop if the robot is close to an obstacle
        if self.odometry.pose.pose.position.x < 1.0:
            self.publish_velocity(0.0, 0.0)
        else:
            # Move forward at a constant linear velocity
            self.publish_velocity(0.2, 0.0)

def main():
    rospy.init_node('multi_robot_controller')

    # Create an instance of the TurtleBot3 class
    t0 = RobotBase('tb3_0')
    t1 = RobotBase('tb3_1')
    t2 = RobotBase('tb3_2')
    t3 = RobotBase('tb3_3')
    t4 = RobotBase('tb3_4')
    t5 = RobotBase('tb3_5')
    t6 = RobotBase('tb3_6')
    t7 = RobotBase('tb3_7')
    t8 = RobotBase('tb3_8')
    t9 = RobotBase('tb3_9')
    t10 = RobotBase('tb3_10')
    #turtlebot3_2 = RobotBase('/tb3_2')
    goals = [
        [-10, 0],
        [-6, 0.5],
        [0.0, 0.2],
        [-10.0, 0.75],
        [-10.0, 1.25],
        [-0.5, -1.0],
        [-0.9, -1.5],
        [-0.1, 0.0],
        [-1.5, -1.5],
        [-0.75, 1.5],
    ]
    rate = rospy.Rate(10) 
 # 10 Hz control loop

    while not rospy.is_shutdown():
        # Update the TurtleBot3's control logic
        
            t1.controller(goals[0])
            t2.controller(goals[1])
            t3.controller(goals[2])
            t4.controller(goals[3])
            t5.controller(goals[4])
            t6.controller(goals[5])
            t7.controller(goals[6])
            t8.controller(goals[7])
            t9.controller(goals[8])
            t10.controller(goals[9])

        #turtlebot3_2.controller([0.4,0.25])[2.3272528804546586, -0.7769478402048842]

            rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
