#! /usr/bin/env python3
# launched as a node under /bat_swarm
#import ros stuff
import rospy
#import messages
from std_msgs.msg import UInt32
from bat_algo.msg import Float32List
import math

NP_ = rospy.get_param('NP')
gen_num_list_ = [0.0] * NP_
off_bots_ = [0.0] * NP_
# sub_list_ = [None] * NP_
move_bat_list_ = [0.0] * NP_

def move_bat(msg):
    global move_bat_list_
    move_bat_list_[ int(msg.data[0]) ] = float(msg.data[1])
    
def gen_checker(msg):
    global sub_list_, gen_num_list_, off_bots_
    # * data[0] -> bat number
    # * data[1] -> gen number
    # * data[2] -> bot's done state
    gen_num_list_[ int(msg.data[0]) ] = float(msg.data[1])
    off_bots_[ int(msg.data[0]) ] = float(msg.data[2])
    # for n in range(NP_):
    #     if off_bots_[n] == 1.0:
    #         sub_list_[n].unregister()
    
def main():
    global NP_, sub_list_, gen_num_list_, off_bots_
    rospy.init_node('gen_checker')
    publisher = rospy.Publisher('common_gen', Float32List, queue_size=1)

    for n in range(NP_):
        name = 'bat_' + str(n+1) + '/cur_gen'
        # sub_list_[n] = 
        rospy.Subscriber(name, Float32List, gen_checker ) # /bat_swarm/bat_1/cur_gen <- global topic name
        rospy.loginfo("gen_checker subscribed to bot %s of %s " %(n+1, NP_))
    for n in range(NP_):
        name = 'bat_' + str(n+1) + '/move_bat'
        # sub_list_[n] = 
        rospy.Subscriber(name, Float32List, move_bat ) # /bat_swarm/bat_1/move_bat <- global topic name
        rospy.loginfo("move_bat subscribed to bot %s of %s " %(n+1, NP_))

    msg = Float32List()
    msg.data = [ float(0), float(0) ]
    publisher.publish(msg)
    
    rate = rospy.Rate(20)
    
    count_list = [False]*NP_
    reach_time = [0.0]*NP_
    
    while not rospy.is_shutdown():
        # average = sum(gen_num_list_)/(NP_-sum(off_bots_))
        average_start = sum(move_bat_list_)/NP_ 
        
        if(average_start==1.0 and count1 == False):
            start_time = rospy.Time.now()
            count1 = True
        
        for n in range(NP_):
            if(off_bots_[n]==1 and count_list[n]== False):
                reach_time[n] = rospy.Time.now()
                count_list[n] = True
    
        average = sum(gen_num_list_)/NP_ 
        
        if math.ceil(average) == math.floor(average): # check if integer
            msg.data =  [ float(average), sum(off_bots_) ]
            # for n in range(NP_):
            #     if off_bots_[n] == 1.0:
            #         gen_num_list_[n] += 1.0
            publisher.publish(msg)
        rospy.loginfo_throttle(period=0.2, msg="avg_gen_num: \033[0;36m%.2f\033[0m  bots done: \033[0;36m%i\033[0m" % (average, sum(off_bots_)))
        
        if sum(off_bots_) == NP_:
            rospy.signal_shutdown(reason="It's Over!!!")
            end_time = rospy.Time.now()
            break
        rate.sleep()
    run_time = end_time.to_sec() - start_time.to_sec()
    rospy.loginfo("\033[0;36m Simulation time taken : %f \033[0m" , run_time) 
    for n in range(NP_):       
        rospy.loginfo("\033[0;36m Time taken for %i bat to reach goal: %f \033[0m" ,n+1, reach_time[n])

if __name__ == '__main__':
    main()
