from matplotlib import pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import math

def R_max_finder(range_info):

    R_max = []
    N_ranges = len(range_info)
    del_angle = 2*np.pi/N_ranges

    for i in range(N_ranges):
        R_test = []
        #print('Break')
        for j in range(math.floor(N_ranges/2)): 
            
            try:
                index = (i + (j- math.floor(N_ranges/4)))%N_ranges
                R_test.append(range_info[index]/(2*math.cos(((j- math.floor(N_ranges/4)))*del_angle)))
                #print((((j- math.floor(N_ranges/4)))*del_angle)*(180/np.pi))
                #print(range_info[index]/2*math.cos(((j- math.floor(N_ranges/4)))*del_angle))
            except:
                print('Error in R_max_finder')
                #R_test[j] = np.inf
                continue
        #print(min(R_test))
        R_max.append(0.9*min(R_test))

    return R_max

def waypoint_finder(Gap_selected,R_max,range_info):
    side_1 = min(Gap_selected)
    side_2 = max(Gap_selected)
    waypoint_distance_from_robot = 0
    waypoint_index = 0
	
    rel_angle = ((side_2 - side_1)%len(range_info))*2*np.pi/len(range_info)

    val = ((range_info[side_1]/range_info[side_2]) + math.cos(rel_angle))*(1/math.sin(rel_angle))

    theta_1 = math.atan(1/val)
    selected_gap_index = (math.floor((theta_1/rel_angle)*(side_2-side_1)) + side_1)%len(range_info) #Angle index of the gap midpoint
    selected_gap_distance = math.sqrt(range_info[side_1]**2 + range_info[side_2]**2 + 2*range_info[side_1]*range_info[side_2]*math.cos(rel_angle))/2 #median of the triangle

    min_distance_from_waypoint_to_goal = np.inf

    start_search_index = (selected_gap_index - math.floor(len(range_info)/4))%len(range_info)#To search only within +- pi/2 from the given gap index


    for i in range(math.floor(len(range_info)/2)):
        can_index = (start_search_index + i)%(len(range_info)) #Candidate angle indices to be swept over
        opt_distance_from_robot_to_can_waypoint = range_info[can_index]*math.cos((2*np.pi/len(range_info))*((can_index-selected_gap_index)%(len(range_info)))) 

        if(opt_distance_from_robot_to_can_waypoint>R_max[can_index]):
            opt_distance_from_robot_to_can_waypoint = R_max[can_index]
        if(opt_distance_from_robot_to_can_waypoint<0):
            continue

        distance_from_waypoint_to_goal = math.sqrt(opt_distance_from_robot_to_can_waypoint**2 + selected_gap_distance**2 - 2*opt_distance_from_robot_to_can_waypoint*selected_gap_distance*math.cos((2*np.pi/len(range_info))*(can_index-selected_gap_index)%(len(range_info))))

        if(distance_from_waypoint_to_goal<min_distance_from_waypoint_to_goal):
            min_distance_from_waypoint_to_goal = distance_from_waypoint_to_goal
            waypoint_distance_from_robot = opt_distance_from_robot_to_can_waypoint
            waypoint_index = can_index

    return waypoint_distance_from_robot,waypoint_index,selected_gap_distance

def waypoint_finder_CODED(Gap_co_ords,R_max,range_info,Robot_pose):

    waypoint_distance_from_robot = 0
    waypoint_index = 0
	
    min_distance_from_waypoint_to_goal = np.inf

    angle_gap = math.atan2(Gap_co_ords[1]-Robot_pose[1],Gap_co_ords[0]-Robot_pose[0]) - Robot_pose[2]

    if(angle_gap>2*np.pi):
        angle_gap = angle_gap-2*np.pi
    
    if(angle_gap<0):
        angle_gap = angle_gap+2*np.pi


    selected_gap_index = math.floor(len(range_info)*(angle_gap)/(2*np.pi)) 
    selected_gap_distance = dist([Robot_pose[0],Robot_pose[1]],[Gap_co_ords[0],Gap_co_ords[1]])
    #start_search_index = (selected_gap_index - math.floor(len(range_info)/4))%len(range_info)#To search only within +- pi/2 from the given gap index

    start_search_index = 0

    for i in range(math.floor(len(range_info))):
        can_index = (start_search_index + i)%(len(range_info)) #Candidate angle indices to be swept over
        opt_distance_from_robot_to_can_waypoint = selected_gap_distance*math.cos((2*np.pi/len(range_info))*((can_index-selected_gap_index)%(len(range_info)))) 

        #opt_distance_from_robot_to_can_waypoint = selected_gap_distance*math.cos(angle_gap)

        if(opt_distance_from_robot_to_can_waypoint>R_max[can_index]):
            opt_distance_from_robot_to_can_waypoint = R_max[can_index]
        if(opt_distance_from_robot_to_can_waypoint<0):
            continue

        distance_from_waypoint_to_goal = math.sqrt(opt_distance_from_robot_to_can_waypoint**2 + selected_gap_distance**2 - 2*opt_distance_from_robot_to_can_waypoint*selected_gap_distance*math.cos((2*np.pi/len(range_info))*(can_index-selected_gap_index)%(len(range_info))))

        if(distance_from_waypoint_to_goal<min_distance_from_waypoint_to_goal):
            min_distance_from_waypoint_to_goal = distance_from_waypoint_to_goal
            waypoint_distance_from_robot = opt_distance_from_robot_to_can_waypoint
            waypoint_index = can_index

    return waypoint_distance_from_robot,waypoint_index,selected_gap_distance


def dist(p1, p2):
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)

def range_smoother(range_info,min,max):
    smooth_array = []
    for i in range(len(range_info)):
        temp_range = range_info[i]

        if(temp_range==min):
            continue
        elif(temp_range>=1.5):
            smooth_array.append(1.5)
        else:
            smooth_array.append(temp_range-0.1)
        ##if(range_info[i]==min):
            ##continue
            ##smooth_array.append(0.2)
        ##elif(range_info[i]>=1.5):
            ##smooth_array.append(1.5)
        ##else:
            ##smooth_array.append(range_info[i]-0.25)
    return smooth_array

def calculate_distance(r_1,r_2,rel_angle):
    dis = math.sqrt(r_1**2 + r_2**2 - 2*r_1*r_2*math.cos(rel_angle))
    return dis


def gaps(range_info):

    Fwd_scan = []
    Bck_scan = []
    Gaps = []

    Robot_dia = 0.2

    dis_threshold = 0.2

    #range_info = 0.5*np.random.randint(5, size=20) #Generating 20 random range readings
    #range_info = [1,2,2,2,2,1,1,1,1,2,2,2,2,1,1,1,1,1,1,1]
    N_ranges = len(range_info)
    del_angle = 2*np.pi/N_ranges

    #Checking for discontinuities in forward scan
    for i in (range(len(range_info))):
        #print((i+1)%N_ranges)
        #if(range_info[i] < 0.1):
            #continue
        if ( (range_info[(i+1)%N_ranges] - range_info[i%N_ranges]) > dis_threshold):
            Fwd_scan.append(i)

    #print(Fwd_scan)

    #Checking for discontinuities in backward scan
    for i in reversed(range(len(range_info))):
        #print((i-1)%N_ranges)
        #if(range_info[i] < 0.1):
            #continue
        if ( (range_info[(i)%N_ranges] - range_info[(i-1)%N_ranges]) > dis_threshold):
            Bck_scan.append(i)

    #print(Bck_scan)


    if(len(Fwd_scan)>0):
        #print('Gaps detected in forward scan')

        for i in range(len(Fwd_scan)):
            min_D = np.inf

            j = (Fwd_scan[i]+1)%N_ranges
            
            for tmp in range(math.floor(N_ranges/2)):
                #print((j+tmp)%N_ranges)
                D = calculate_distance(range_info[Fwd_scan[i]],range_info[(j+tmp)%N_ranges],tmp*del_angle)

                if(D<min_D):
                    min_D = D
                    min_ref = (j+tmp)%N_ranges

            if(min_D>Robot_dia):
                Gaps.append([Fwd_scan[i],min_ref])
        
        print(Gaps)

    if(len(Bck_scan)>0):
        #print('Gaps detected in backward scan')

        for i in range(len(Bck_scan)):
            min_D = np.inf

            j = (Bck_scan[i]-1)%N_ranges

            for tmp in (range(math.floor(N_ranges/2))):
                #print((j-tmp)%N_ranges)

                D = calculate_distance(range_info[Bck_scan[i]],range_info[(j-tmp)%N_ranges],tmp*del_angle)

                if(D<min_D):
                    min_D = D
                    min_ref = (j-tmp)%N_ranges
            
            if(min_D>Robot_dia):
                Gaps.append([Bck_scan[i],min_ref])

        #print(Gaps)
        return Gaps
    
def Optimal_Gap_finder(Gaps,range_info,Target_navigation_point,robot_pose):

    min_gap_distance = np.infty
    gap_index = 0

    for i in range(length(Gaps)):

        Gap_selected = Gaps[i]
        side_1 = min(Gap_selected)
        side_2 = max(Gap_selected)
        #waypoint_distance_from_robot = 0
	
        rel_angle = ((side_2 - side_1)%len(range_info))*2*np.pi/len(range_info)

        val = ((range_info[side_1]/range_info[side_2]) + math.cos(rel_angle))*(1/math.sin(rel_angle))

        theta_1 = math.atan(1/val)
        selected_gap_index = (math.floor((theta_1/rel_angle)*(side_2-side_1)) + side_1)%len(range_info) #Angle index of the gap midpoint
        selected_gap_distance = math.sqrt(range_info[side_1]**2 + range_info[side_2]**2 + 2*range_info[side_1]*range_info[side_2]*math.cos(rel_angle))/2 #median of the triangle
        
        #side_1_position = robot_pose[0] + 

        distance_to_goal = math.sqrt()
        if(selected_gap_distance<min_gap_distance):
            min_gap_distance = selected_gap_distance
            gap_index = i

    return Gaps[i]

