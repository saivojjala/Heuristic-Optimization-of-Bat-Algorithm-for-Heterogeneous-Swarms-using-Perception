#! /usr/bin/env python3

# import ros stuff
import dis
import actionlib
import rospy

# import ros message
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import * # Float32, Float64

# from tf import transformations
from tf import transformations

# import ros service
from std_srvs.srv import *
from bat_algo.msg import Float32List
from bat_algo.msg import GoToPointAction, GoToPointGoal, GoToPointFeedback, GoToPointResult

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import math

import random 
import numpy as np

des_pos_x_ = 0.0
des_pos_y_ = 0.0

dist_precision_ = 0.2
position_ = Point()
yaw_ = 0.0

act_client_go_to_point_ = None

cmd_vel_publisher_ = None
gen_publisher_ = None
fit_pos_publisher_ = None
# off_publisher_ = None

laser_val_ = 10.0
common_gen_ = 0

D_ = rospy.get_param('/bat_swarm/D')   #dimension
NP_ = rospy.get_param('/bat_swarm/NP')  #population size 
N_Gen_ = 0  #generations
A_ = 0.00  #loudness
alpha_ = 0.00 
r_init_ = 0.00
gamma_ = 0.00 
r_ = 0.00  #pulse rate
Qmin_ = 0.00  #frequency min
Qmax_ = 0.00  #frequency max
Lower_ = 0.00  #lower bound
Upper_ = 0.00  #upper bound
off_bots_ = 0  #number of bots that have reached close to goal point

f_min_ = 0.0  #minimum fitness

Lb_ = [0.00] * D_  #lower bound
Ub_ = [0.00] * D_  #upper bound
Q_ = 0.00 #frequency

v_ = [0.00 for i in range(D_)]  #velocity
Cur_Pos_ = [0.00 for i in range(D_)] #population of solutions
Fitness_ = 0.00  #fitness
best_ = [0.00] * D_  #position of the best solution (x, y) values

name_space_ = rospy.get_namespace()



def simplebounds(val, lower, upper):
    if val < lower:
        val = lower
    if val > upper:
        val = upper
    return val

def move_bat():
    global fit_pos_publisher_, gen_publisher_, D_, NP_, N_Gen_, A_, r_, Qmin_, Qmax_, Lower_, Upper_, v_, \
        Fitness_, best_, Cur_Pos_, f_min_, Q_, Ub_, Lb_, common_gen_, position_,  \
        des_pos_x_, des_pos_y_, alpha_, gamma_, off_bots_

    Nxt_Pos = [0.0 for i in range(D_)]
    # get distance between goal and Cur_Pos
    distance_to_goal = math.sqrt(math.pow(des_pos_x_ - Cur_Pos_[0], 2)\
                               + math.pow(des_pos_y_ - Cur_Pos_[1], 2))
    # calc fitness in the initial starting positions
    Fnew = fitness_function(distance_to_goal)
    Fitness_ = Fnew
    rospy.loginfo("[%s] Initial Fitness: %.2f " % (name_space_,Fnew))

    bat_index = int(name_space_[-2])-1
    msg = Float32List()
    msg.data = [ bat_index, Fnew, Cur_Pos_[0], Cur_Pos_[1] ]
    fit_pos_publisher_.publish(msg)
    gen_msg = Float32List()
    state = 0.0     # * if bot is at goal becomes 1
    result = GoToPointResult()
    for t in range(N_Gen_):
        if distance_to_goal < 0.5*(off_bots_+1): # ? added brackets around "off_bots + 1"
            state = 1.0
        # publish gen number for the comon gen update and completion state
        gen_msg.data = [ float(bat_index), float(t) , state]
        gen_publisher_.publish(gen_msg)
        
        if state == 1.0:
            break

        rospy.loginfo("[%s] Gen Number: %s" %(name_space_, t))

        # ? wait for all bots
        while t != common_gen_:
            continue
        # ? all bots in sync continue

        Cur_Pos_[0] = position_.x
        Cur_Pos_[1] = position_.y

        rnd = np.random.uniform(0, 1)
        Q_ = Qmin_ + (Qmax_ - Qmin_) * rnd         # ? Decide random bounded Frequency

        # rospy.loginfo("[%s] Frequency: %s" %(name_space_, Q_))

        for i in range(D_):
            v_[i] = v_[i] + (Cur_Pos_[i] - best_[i]) * Q_              # ? head toward the best_position pos on the axis
            Nxt_Pos[i] = Cur_Pos_[i] + v_[i]                           # ? updated position
            Nxt_Pos[i] = simplebounds(Nxt_Pos[i], Lb_[i], Ub_[i])      # ? bound it

        rnd = np.random.uniform(0, 1)

        if rnd > r_:
            for i in range(D_):
                epsilon = 0.8
                Nxt_Pos[i] = best_[i] + epsilon * A_ * random.gauss(0, 1)  # ? travel towards some place in the vicinity of best position
                Nxt_Pos[i] = simplebounds(Nxt_Pos[i], Lb_[i], Ub_[i])
            
        # Movement code below
        # move robot to Nxt_Pos[0], Nxt_Pos[1]
        distance_to_nxt_pos = abs(math.sqrt( ((Nxt_Pos[0]-Cur_Pos_[0])**2) +
                                          ((Nxt_Pos[1]-Cur_Pos_[1])**2) ) )
        
        if distance_to_nxt_pos > 0.0:
            result = go_to_point(Nxt_Pos)
            distance_to_goal = result.distance
            Nxt_Pos[0] = result.x
            Nxt_Pos[1] = result.y
        else:
            distance_to_goal = math.sqrt(math.pow(des_pos_x_ - Cur_Pos_[0], 2)\
                                       + math.pow(des_pos_y_ - Cur_Pos_[1], 2))
        #* calc fitness after robot reaches goal point
        Fnew = fitness_function(distance_to_goal)
        #* update fittness and corresponding position on the best_bat server
        msg.data = [ bat_index, Fnew, Nxt_Pos[0], Nxt_Pos[1] ]
        #* publish Fnew value
        fit_pos_publisher_.publish(msg)
        rospy.loginfo("[%s] Fitness: %.4f\tA_: %.7f\tr_:%.7f " % (name_space_, Fnew, A_, r_))
        
        # ! rnd = np.random.random_sample() #! DO NOT ADD THIS AS IT NEEDS TO BE SAME AS ABOVE VALUE FOR rnd

        if (Fnew <= Fitness_) and (rnd < A_):
            for i in range(D_):
                Cur_Pos_[i] = Nxt_Pos[i]  # position is better so we decide to stay there
            Fitness_ = Fnew
            A_ = alpha_ * A_
            r_ = r_init_* (1-math.exp(-gamma_*(t+1)))
        else:       # New position is not good
            # move the robot back to previous position
            if distance_to_nxt_pos > dist_precision_:
                if (rnd > A_):
                    rospy.loginfo("[%s] Randomly going back to (%.2f, %.2f)" %(name_space_ ,Cur_Pos_[0], Cur_Pos_[1]))
                if (Fnew > Fitness_):
                    rospy.loginfo("[%s] Poor fitness going back to (%.2f, %.2f)" %(name_space_ ,Cur_Pos_[0], Cur_Pos_[1]))
                result = go_to_point(Cur_Pos_)
                if Cur_Pos_[0] != result.x and Cur_Pos_[1] != result.y:
                    Cur_Pos_[0] = result.x
                    Cur_Pos_[1] = result.y

            # end of move back to previous position
    # print(f_min_)

def go_to_point(Pos):
    global act_client_go_to_point_, Cur_Pos_, name_space_, dist_precision_
    if Pos[0] != Cur_Pos_[0] and Pos[1] != Cur_Pos_[1]:
        rospy.loginfo('[%s] Moving robot to Nxt_Pos:[ %.2f , %.2f ] from [ %.2f , %.2f ] ' \
                     % (name_space_, Pos[0], Pos[1], Cur_Pos_[0], Cur_Pos_[1]))
    goal = GoToPointGoal()
    goal.x = Pos[0]
    goal.y = Pos[1]
    act_client_go_to_point_.send_goal(goal)#, feedback_cb=feedback_cb)
    act_client_go_to_point_.wait_for_result()
    # rospy.loginfo('[%s] Robot reached Nxt_Pos' %(name_space_))
    result = act_client_go_to_point_.get_result()
    
    rospy.loginfo("[%s] Currently at (%.2f, %.2f)" %(name_space_ ,result.x, result.y))
        
    return result

def fitness_function(distance):
    global laser_val_
    # laser_val_ = Float64()
    val = laser_val_
    # calc distance betwen (Nxt_Pos[0], Nxt_Pos[1]) -> (des_pos_x, des_pos_y)
    # pub pos_0 , pos1 , fitness
    # fitness = distance + 4.00/(laser_val_)    # fitness equation
    fitness = distance # + pow(val-0.34, -4)   # fitness equation
    return fitness

def min_laser_clbk(val):
    global laser_val_
    laser_val_ = val.data

def best_bat_clbk(msg):
    global f_min_, best_
    f_min_ = msg.data[0]
    best_[0] = msg.data[1]
    best_[1] = msg.data[2]

def gen_checker_clbk(msg):
    global common_gen_, name_space_, off_bots_
    common_gen_ = msg.data[0]
    off_bots_ = msg.data[1]
    # rospy.loginfo_throttle(period=2, msg='[%s] Gen updated to [%s]' %(name_space_, common_gen_))

# Odometry Callback
def odom_clbk(msg):
    global position_
    global yaw_
    # position
    position_ = msg.pose.pose.position
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def main():
    global fit_pos_publisher_, gen_publisher_, act_client_go_to_point_, \
        des_pos_x_, des_pos_y_, Cur_Pos_, D_, NP_, N_Gen_, \
        A_, r_, Qmin_, Qmax_, Lower_, Upper_, Lb_, Ub_, position_, \
        alpha_, r_init_, gamma_, common_gen_, off_bots_
        
    rospy.loginfo("Starting_node: move_bat")
    rospy.init_node("move_bat")

    state_msg = ModelState()
    # namespace = str(rospy.get_namespace()) # /bat_swarm/bat_1
    
    bot_number = rospy.get_param('robot_number')
    bot_model_name = "bat_" + str(bot_number)
    state_msg.model_name = bot_model_name
    state_msg.pose.position.x =  rospy.get_param('initial_x')
    state_msg.pose.position.y =  rospy.get_param('initial_y')
    state_msg.pose.position.z =  0.1
    # state_msg.pose.orientation.x = 0.0
    # state_msg.pose.orientation.y = 0.0
    # state_msg.pose.orientation.z = 0.0
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state =  rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        
    act_client_go_to_point_ = actionlib.SimpleActionClient( 'go_to_point', GoToPointAction )

    gen_publisher_ = rospy.Publisher('cur_gen', Float32List, queue_size=1, latch=True)
    fit_pos_publisher_ = rospy.Publisher('fit_pos', Float32List, queue_size=4)
    move_bat_publisher = rospy.Publisher('move_bat', Float32List, queue_size=1, latch=True)
    # off_publisher_ = rospy.Publisher('off_bot', Bool, queue_size=1)

    sub_laser = rospy.Subscriber('min_laser', Float64, min_laser_clbk)
    sub_best_bat = rospy.Subscriber('/bat_swarm/best_bat_data', Float32List, best_bat_clbk)
    sub_gen_check = rospy.Subscriber('/bat_swarm/common_gen', Float32List, gen_checker_clbk)
    sub_odom = rospy.Subscriber('odom', Odometry, odom_clbk)

    des_pos_x_ = rospy.get_param('/bat_swarm/des_pos_x')
    des_pos_y_ = rospy.get_param('/bat_swarm/des_pos_y')

    Cur_Pos_[0] = rospy.get_param('initial_x')
    Cur_Pos_[1] = rospy.get_param('initial_y')

    position_.x = rospy.get_param('initial_x')
    position_.y = rospy.get_param('initial_y')  # Kind of works

    N_Gen_ = rospy.get_param('/bat_swarm/N_Gen')
    Qmin_ = rospy.get_param('/bat_swarm/Qmin')
    Qmax_ = rospy.get_param('/bat_swarm/Qmax')
    Lower_ = rospy.get_param('/bat_swarm/Lower')
    Upper_ = rospy.get_param('/bat_swarm/Upper')
    
    for i in range(D_):
        Lb_[i] = Lower_
        Ub_[i] = Upper_

    A_ = rospy.get_param('A')
    r_ = rospy.get_param('r')
    alpha_ = rospy.get_param('/bat_swarm/alpha')
    r_init_ = rospy.get_param('r')      # rospy.get_param('/bat_swarm/r_init')
    gamma_ = rospy.get_param('/bat_swarm/gamma')

    bat_index = int(name_space_[-2])-1
    move_bat_msg = Float32List()
    move_bat_msg.data = [ float(bat_index), float(1.0) ]
    move_bat_publisher.publish(move_bat_msg)

    move_bat()

    rospy.loginfo("[%s] \033[0;32mReached Goal !!!\033[0m" %(name_space_))
    
    sub_laser.unregister()
    sub_best_bat.unregister()
    
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1 )
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_vel_pub.publish(twist_msg)
    # sub_gen_check.unregister()
    
    bat_index = int(name_space_[-2])-1
    gen_msg = Float32List()
    rate = rospy.Rate(10)
    
    while off_bots_ < NP_ :
        gen_updated = common_gen_ + 1
        rospy.loginfo_throttle_identical(period = 30 ,  msg="[%s] \033[0;33mManual Gen Update: %f \033[0m" %(name_space_, gen_updated))
        # rospy.loginfo_throttle_identical("[%s] Manual Gen Update: %.2f" %(name_space_, gen_updated))
        gen_msg.data = [ float(bat_index), float(common_gen_+1) , 1.0]
        gen_publisher_.publish(gen_msg)
        rate.sleep()
    rospy.spin()

if __name__ == "__main__":
    main()