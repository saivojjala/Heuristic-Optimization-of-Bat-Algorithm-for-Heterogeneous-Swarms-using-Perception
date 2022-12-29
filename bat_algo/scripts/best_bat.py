#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message

# import ros service
from std_srvs.srv import *
from bat_algo.msg import Float32List
from std_msgs.msg import Float32
import numpy as np

f_min_ = 0.0

D_ = rospy.get_param('D')
NP_ = rospy.get_param('NP')

Nxt_Pos_list_ = [[0.0 for i in range(D_)] for j in range(NP_)]
best_ = [0.0, 0.0]# for i in range(D_)]

fitness_list_ = [1000.0] * NP_

publisher_ = None



# /bat_swarm/bat_1/fit_pos Float32List
# list of all the fitness

def update_vars(msg):
    global fitness_list_, Nxt_Pos_list_, D_
    fitness_list_[int(msg.data[0])] = msg.data[1]
    for i in range(D_):
        Nxt_Pos_list_[int(msg.data[0])][i] = msg.data[i+2]

def main():
    global f_min_, Nxt_Pos_list_, best_, D_, NP_, publisher_

    rospy.init_node('best_bat')

    publisher_ = rospy.Publisher('best_bat_data', Float32List, latch=True, queue_size=1)
    
    for n in range(NP_):    
        name = 'bat_' + str(n+1) + '/fit_pos'
        rospy.Subscriber(name, Float32List, update_vars )
        rospy.loginfo('best_bat: Subscribed to %s' % name)

    rate = rospy.Rate(20)

    msg = Float32List()

    while not rospy.is_shutdown() :
        f_min_ = fitness_list_[0]  # choose first bot fitness to start
        
        for i in range(len(fitness_list_)):
            if fitness_list_[i] < f_min_:
                f_min_ = fitness_list_[i]
                best_[0] = Nxt_Pos_list_[i][0]
                best_[1] = Nxt_Pos_list_[i][1]
                
        msg.data = [f_min_, best_[0], best_[1]]
        rospy.loginfo_throttle(period=0.2, msg=" min_fit: \033[0;34m%.3f\033[0m, X: \033[0;34m%.3f\033[0m,  Y: \033[0;34m%.3f\033[0m" % ( msg.data[0], msg.data[1], msg.data[2]) )
        publisher_.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()