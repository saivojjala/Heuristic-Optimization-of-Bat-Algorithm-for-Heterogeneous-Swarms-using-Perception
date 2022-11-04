#! /usr/bin/env python3

# import ros stuff
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

# std python libraries
import math
import random 
import numpy as np

des_pos_x_ = 0.0
des_pos_y_ = 0.0

# robot state variables
active_ = True

position_ = Point()
yaw_ = 0.0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = 0
desired_position_.y = 0
desired_position_.z = 0

# parameters
yaw_precision_ = math.pi / 40 #45  # +/- 4 degree allowed
dist_precision_ = 0.02

# Publishers
cmd_vel_publisher_ = None
gen_publisher_ = None
fit_pos_publisher_ = None

laser_val_ = 10.0
common_gen_ = 0

D_ = rospy.get_param('/bat_swarm/D')   #dimension
NP_ = rospy.get_param('/bat_swarm/NP')  #population size 
N_Gen_ = 0  #generations
A_ = 0.00  #loudness
r_ = 0.00  #pulse rate
Qmin_ = 0.00  #frequency min
Qmax_ = 0.00  #frequency max
Lower_ = 0.00  #lower bound
Upper_ = 0.00  #upper bound

f_min_ = 0.0  #minimum fitness

Lb_ = [0.00] * D_  #lower bound
Ub_ = [0.00] * D_  #upper bound
Q_ = 0.00 #frequency

v_ = [0.00 for i in range(D_)]  #velocity
Cur_Pos_ = [0.00 for i in range(D_)] #population of solutions
Fitness_ = 0.00  #fitness
best_ = [0.00] * D_  #position of the best solution (x, y) values

name_space_ = rospy.get_namespace()

def change_state(state):
    global state_
    state_ = state
    rospy.loginfo('[%s] State changed to [%s]' %(name_space_, state_))

def fix_yaw(des_pos):
    global yaw_, cmd_vel_publisher_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = math.fabs(desired_yaw) - math.fabs(yaw_)
    yaw_val = math.fabs(err_yaw)/math.fabs(desired_yaw)
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = -yaw_val if err_yaw < 0 else yaw_val  # >

    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        twist_msg.angular.z = 0
        # rospy.loginfo('Yaw error: [%s]' % err_yaw)
        change_state(1)

    cmd_vel_publisher_.publish(twist_msg)

def go_straight(des_pos):
    global yaw_, cmd_vel_publisher_, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))

    twist_msg = Twist()
    twist_msg.linear.x = 0.00
    if err_pos > dist_precision_:
        twist_msg.linear.x = 0.2 *err_pos
        change_state(2)
    # if laser_val_ < 0.45:
    #     twist_msg.linear.x = 0.00
    #     rospy.logwarn( 'Too close to obstace: [%s]' % laser_val_)
    #     change_state(2)
    # state change conditions if somehow error increased
    if math.fabs(err_yaw) > yaw_precision_:
        # print('Yaw error: [%s]' % err_yaw)
        change_state(0)

    cmd_vel_publisher_.publish(twist_msg)

def done(state_):
    global cmd_vel_publisher_, active_
    active_ = False
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    cmd_vel_publisher_.publish(twist_msg)

def go_to_point_clbk(nxt_pos):
    global state_, active_, desired_position_, Cur_Pos_, des_pos_x_, des_pos_y_
    rospy.loginfo('[%s] Moving robot to Nxt_Pos:[ %s , %s ] from [ %s , %s ]' % (name_space_, nxt_pos[0], nxt_pos[1], Cur_Pos_[0], Cur_Pos_[1]))
    rospy.loginfo("[%s] Before setting up vars" % name_space_)
    desired_position_.x =  nxt_pos[0]
    desired_position_.y =  nxt_pos[1]
    state_ = 0
    active_ = True
    rospy.loginfo("[%s] Before while Loop" % name_space_)
    rate2 = rospy.Rate(20)
    rospy.loginfo("[%s] While Loop" % name_space_)
    while(active_ == True):
        rospy.loginfo_throttle(period=5, msg= "[%s] active_: %s" %(name_space_, active_))
        if state_ == 0:
            rospy.loginfo_throttle(period=1, msg = "[%s] Fix_yaw" % name_space_)
            fix_yaw(desired_position_)
        elif state_ == 1:
            rospy.loginfo_throttle(period=1, msg = "[%s] Go_straight" % name_space_)
            go_straight(desired_position_)
        elif state_ == 2:
            rospy.loginfo(msg = "[%s] Done" % name_space_) #  _throttle(period=0.5, 
            done(state_)

        rate2.sleep()
    
    rospy.loginfo('[%s] Robot reached Nxt_Pos' %(name_space_))
    result = math.sqrt(math.pow(des_pos_x_ - desired_position_.x, 2) + math.pow(des_pos_y_ - desired_position_.y, 2))
    return result
    
def simplebounds(val, lower, upper):
    if val < lower:
        val = lower
    if val > upper:
        val = upper
    return val

def move_bat():
    global fit_pos_publisher_, gen_publisher_, D_, NP_, N_Gen_, A_, r_, Qmin_, Qmax_, Lower_, Upper_, v_, \
        Fitness_, best_, Cur_Pos_, f_min_, Q_, Ub_, Lb_, common_gen_, \
        des_pos_x_, des_pos_y_

    Nxt_Pos = [0.0 for i in range(D_)]
    # get distance between goal and Cur_Pos
    distance_to_goal = math.sqrt(math.pow(des_pos_x_ - Cur_Pos_[0], 2)\
                               + math.pow(des_pos_y_ - Cur_Pos_[1], 2))
    # calc fitness in the initial starting positions
    Fnew = fitness_function(distance_to_goal)

    rospy.loginfo("[%s] Initial Fitness: %s " % (name_space_,Fnew))

    bat_index = int(name_space_[-2])-1
    msg = Float32List()
    msg.data = [ bat_index, Fnew, Cur_Pos_[0], Cur_Pos_[1] ]
    fit_pos_publisher_.publish(msg)
    gen_msg = Float32List()

    for t in range(N_Gen_):
        # publish gen number for the 
        gen_msg.data = [ bat_index, t ]
        gen_publisher_.publish(gen_msg)

        rospy.loginfo("[%s] Gen Number: %s" %(name_space_, t))

        while True:
            if t == common_gen_:
                break
            else:
                rospy.spin()
        # wait for all bots
        # rospyGen_Num
        # all bots in sync continue

        rnd = np.random.uniform(0, 1)
        Q_ = Qmin_ + (Qmax_ - Qmin_) * rnd         # Decide random bounded Frequency

        rospy.loginfo("[%s] Frequency: %s" %(name_space_, Q_))

        for i in range(D_):                                             # For x and y axis each
            v_[i] = v_[i] + (Cur_Pos_[i] - best_[i]) * Q_               # head toward the best_position pos on the axis
            Nxt_Pos[i] = Cur_Pos_[i] + v_[i]                           # updated position
            Nxt_Pos[i] = simplebounds(Nxt_Pos[i], Lb_[i], Ub_[i])      # bound it

        rnd = np.random.random_sample()

        if rnd > r_:
            for i in range(D_):
                Nxt_Pos[i] = best_[i] + 0.1 * random.gauss(0, 1)  # travel towards some place in the vicinity of best position
                Nxt_Pos[i] = simplebounds(Nxt_Pos[i], Lb_[i], Ub_[i])

        # Movement code below
        # move robot to Nxt_Pos[0], Nxt_Pos[1]
        distance = abs(math.sqrt( ((Nxt_Pos[0]-Cur_Pos_[0])**2) +
                                          ((Nxt_Pos[1]-Cur_Pos_[1])**2) ) )
        if distance > 0.0:
            distance_to_goal = go_to_point_clbk(Nxt_Pos)
        else:
            distance_to_goal = 0.0
            
        # calc fitness after robot reaches goal point
        Fnew = fitness_function(distance_to_goal)
        msg.data = [ bat_index, Fnew, Nxt_Pos[0], Nxt_Pos[1] ]
        fit_pos_publisher_.publish(msg)

        rospy.loginfo("[%s] Fitness: %s " % (name_space_,Fnew))

        rnd = np.random.random_sample()

        if (Fnew <= Fitness_) and (rnd < A_):
            for i in range(D_):
                Cur_Pos_[i] = Nxt_Pos[i]  # position is better so we decide to stay there
            Fitness_ = Fnew
            # A_ = alpha * A_
            # r_ = r_init_* (1-math.exp(-gamma*t))
        else:       # New position is not good`
            # move the robot back to previous position
            go_to_point_clbk(Cur_Pos_)
            # end of move back to previous position

        # publish Fnew value

        # End of publish Fnew value
    # print(f_min_)

def fitness_function(distance):
    global laser_val_
    # laser_val_ = Float64()
    val = laser_val_
    # calc distance betwen (Nxt_Pos[0], Nxt_Pos[1]) -> (des_pos_x, des_pos_y)
    # pub pos_0 , pos1 , fitness
    # fitness = distance + 4.00/(laser_val_)    # fitness equation
    fitness = distance + (5.0*val+10.0)/(math.pow(val, 2) -4.0)   # fitness equation
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
    global common_gen_
    common_gen_ = msg.data

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
    global fit_pos_publisher_, gen_publisher_, cmd_vel_publisher_, \
        des_pos_x_, des_pos_y_, Cur_Pos_, D_, NP_, N_Gen_, \
        A_, r_, Qmin_, Qmax_, Lower_, Upper_, Lb_, Ub_
    rospy.loginfo("Starting_node: move_bat")
    rospy.init_node("move_bat")

    # act_client_go_to_point_ = actionlib.SimpleActionClient( 'go_to_point', GoToPointAction )

    gen_publisher_ = rospy.Publisher('cur_gen', Float32List, queue_size=1)
    fit_pos_publisher_ = rospy.Publisher('fit_pos', Float32List, queue_size=4)
    cmd_vel_publisher_ = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.Subscriber('min_laser', Float64, min_laser_clbk)
    rospy.Subscriber('/bat_swarm/best_bat_data', Float32List, best_bat_clbk)
    rospy.Subscriber('/bat_swarm/common_gen', UInt32, gen_checker_clbk)
    sub_odom = rospy.Subscriber('odom', Odometry, odom_clbk, queue_size=2)

    des_pos_x_ = rospy.get_param('/bat_swarm/des_pos_x')
    des_pos_y_ = rospy.get_param('/bat_swarm/des_pos_y')

    Cur_Pos_[0] = rospy.get_param('initial_x')
    Cur_Pos_[1] = rospy.get_param('initial_y')

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

    move_bat()

if __name__ == "__main__":
    main()