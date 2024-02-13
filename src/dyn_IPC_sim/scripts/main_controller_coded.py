#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,TransformStamped
from tf.transformations import euler_from_quaternion
import math
import numpy as np
from detect_gaps import gaps,range_smoother,R_max_finder,waypoint_finder,waypoint_finder_CODED
from sensor_msgs.msg import LaserScan
import time

global pose,scan
pose = [0,0,0]
scan = LaserScan()

## quaternion to euler
def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)
########################

## Odometry callback
#def callback_vicon(data):
    #global pose
    #print('Pose obtained from Vicon')
    #x  = data.transform.rotation.x
    #y  = data.transform.rotation.y
    #z  = data.transform.rotation.z
    #w  = data.transform.rotation.w
    #pose = [data.transform.translation.x, data.transform.translation.y, quat2euler(x,y,z,w)[2]]
########################

def RobotPose(data):
    global pose

    pose = [data.pose.pose.position.x,data.pose.pose.position.y,euler_from_quaternion([data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w])[2]]

## LiDAR scan
def callback_LiDAR(data):
    global scan 
    scan = data
    #print('Scan obtained from LiDAR')
    
## Euclidean Distance
def dist(p1, p2):
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)
########################

def bearing(p1, p2):
    return ( math.atan2(p2[1]-p1[1],p2[0]-p1[0]) )
########################

def signum(p1):
    if(p1>=0):
        return 1
    else:
        return -1


def fetch_gaps(scan):

    Gaps = gaps(range_smoother(scan.ranges,scan.range_min,scan.range_max))
    #Gaps = gaps(scan.ranges)
    return Gaps


def store_data_pose_plan(vector1,vector2):
    #data = f"{vector1[0]:.4f} {vector1[1]:.4f} {vector1[2]:.4f} {vector2[0]:.4f} {vector2[1]:.4f} {vector2[2]:.4f}\n"

    with open("data_pose_plan.txt", "a+") as file_pose_plan:
        file_pose_plan.write("{}\t{}\t{}\t{}\t{}\t{}\n".format(vector1[0],vector1[1],vector1[2],vector2[0],vector2[1],vector2[2]))

def store_data_control_inputs(vector1):
    #data = f"{vector1[0][0]:.4f} {vector1[1][0]:.4f} {vector1[2][0]:.4f}\n"

    with open("data_control.txt", "a+") as file_control_inputs:
        file_control_inputs.write("{}\t{}\n".format(vector1[0],vector1[1]))


def fetch_waypoint(pose,scan,current_Gap):
    #global pose,scan

    #Gaps = gaps(range_smoother(scan.ranges,scan.range_min,scan.range_max))
    R_max_set = R_max_finder(range_smoother(scan.ranges,scan.range_min,scan.range_max))
    #R_max_set = R_max_finder(scan.ranges)
    [waypoint_distance,waypoint_index,selected_gap_distance] = waypoint_finder_CODED(current_Gap,R_max_set,range_smoother(scan.ranges,scan.range_min,scan.range_max),pose)
    #[waypoint_distance,waypoint_index,selected_gap_distance] = waypoint_finder_CODED(current_Gap,R_max_set,scan.ranges,pose)
    N_ranges = len(R_max_set)
    del_theta_measure = 2*np.pi/N_ranges


    way_x = pose[0] + waypoint_distance*math.cos((waypoint_index*del_theta_measure) + 1*pose[2])
    way_y = pose[1] + waypoint_distance*math.sin((waypoint_index*del_theta_measure) + 1*pose[2])

    way = [way_x,way_y]
    
    return way,selected_gap_distance

## Main Node
def controller():

    global pose,scan
    
    #Robot_x = pose[0]
    #Robot_y = pose[1]
    #Robot_alpha = pose[2]
    #time.sleep(5)

    rospy.init_node('main_controller', anonymous=True)
    #rospy.Subscriber('/vicon/tb3_7/tb3_7', TransformStamped, callback_vicon) #Insert Vicon Subscriber here
    rospy.Subscriber('/tb3_0/odom',Odometry,RobotPose)
    rospy.Subscriber('/tb3_0/scan',LaserScan,callback_LiDAR) #Insert range scanner here
    pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=10) #Insert velocity publisher here
    velocity_msg = Twist()
    rate = rospy.Rate(4)


    while(len(scan.ranges)==0):
        print('First scan not obtained yet')
        continue
    while(len(pose)==0):
        print('First pose not obtained yet')
        continue
    #Fetching the initial waypoint
    #way = fetch_waypoint(pose,scan)
    #print(way)
    #way = [0.5,0]

    integral_action = 0
    total_plan_time = 0
    planning_instances = 0

    
    selected_gap_distance = np.inf
    Gap_list = [[0.1,0.85],[0.5,0.0]] #To be added from the workspace, final entry should be target point of navigation
    
    #Gap_list = [[-1.077,0.347],[0.0171,-0.0668],[0.948,0.407],[1.57,-0.207]]
    #Gaps = fetch_gaps(scan)
    max_count = len(Gap_list)
    current_gap_index = 0
    current_target_gap = Gap_list[0]
    
    while not rospy.is_shutdown():
        
        if(current_gap_index>(len(Gap_list)-1)):
            break

        if(dist([pose[0],pose[1]],[Gap_list[current_gap_index][0],Gap_list[current_gap_index][1]])<0.1):
            current_gap_index = current_gap_index+1
            continue

        current_target_gap = Gap_list[current_gap_index]

        K_1 = 0.2
        K_2 = 0.3
        K_3 = 0

        planning_instances = planning_instances+1

        Robot_position = [pose[0],pose[1]]
        
        #if(dist(way,Robot_position)<0.1):
            #print(way)
            #way = fetch_waypoint(pose,scan)
            #integral_action = 0
            #continue
            #break
        #Gaps = fetch_gaps(scan)

        #if(not Gaps):
            #continue
        #else:
            #prev_gaps = Gaps

        start_plan_time = time.time()
        way,selected_gap_distance = fetch_waypoint(pose,scan,current_target_gap)
        end_plan_time = time.time()
        total_plan_time = total_plan_time + (end_plan_time-start_plan_time)

        avg_waypoint_planning_duration = total_plan_time/planning_instances 
        
        #way = Gap_list[current_gap_index]
        R = dist(way,Robot_position)
        rel_bearing = pose[2] - bearing(way,Robot_position)
        print(way)
        #To bring the relative bearing values in the range of [pi,-pi)
        if(rel_bearing>np.pi):
            rel_bearing = rel_bearing - 2*np.pi
            #continue
        
        if(rel_bearing<-np.pi):
            rel_bearing = rel_bearing + 2*np.pi 
            #continue

        if(math.cos(rel_bearing)<0):
            S = rel_bearing - signum(rel_bearing)*np.pi
        else:
            S = rel_bearing

        integral_action = integral_action + (-K_3*signum(S)*0.1) #rospy.Rate is set at 10 Hz
        linear_vel = -K_1*math.tanh(R)*signum(math.cos(rel_bearing))
        velocity_msg.linear.x = linear_vel


        if(R>0.1):
            angular_vel = -K_2*(abs(S)**0.5)*signum(S) + integral_action + K_1*signum(math.cos(rel_bearing))*math.sin(rel_bearing)*math.tanh(R)/R
        else:
            angular_vel = -K_2*(abs(S)**0.5)*signum(S) + integral_action + K_1*signum(math.cos(rel_bearing))*math.sin(rel_bearing)
        angular_vel = -K_2*(abs(S)**0.5)*signum(S) + integral_action + K_1*signum(math.cos(rel_bearing))*math.sin(rel_bearing)*math.tanh(R)/R
        velocity_msg.angular.z = angular_vel
        pub.publish(velocity_msg)

        #store_data_pose_plan([pose[0],pose[1],pose[2]],[way[0],way[1],R])
        #store_data_control_inputs([linear_vel,angular_vel])

        rate.sleep()
########################

    velocity_msg.linear.x = 0
    velocity_msg.angular.z = 0
    pub.publish(velocity_msg)

    #file_storage = open("Experiment Data","a+")
    #file_storage.write("Total Computational time for waypoint planning:{}, and the average time taken for computing waypoints:{}".format(total_plan_time,avg_waypoint_planning_duration))
    #file_storage.close()
if __name__ == '__main__':
    try:
        time.sleep(8)
        controller()
    except rospy.ROSInterruptException:
        pass
