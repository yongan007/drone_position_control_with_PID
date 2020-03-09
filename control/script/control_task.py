#!/usr/bin/env python
import rospy
import numpy as np

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from dynamic_reconfigure.msg import Config



def sub_and_pub():
    #set offbaord mode and turn on arm
    arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
    mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    mode = arming_srv(value=True)
 
    rospy.init_node('control', anonymous=True)
    cur_pos_topic = '/mavros/local_position/pose'
    goal_topic = '/goal'
    cur_vel_topic = '/mavros/local_position/velocity_local'
    check_topic = '/mavros/setpoint_velocity/cmd_vel'
    dynamic_reconfigure = '/dynamic_tutorials/parameter_updates'

    rospy.Subscriber(dynamic_reconfigure,Config, update_fields)
    rospy.Subscriber(goal_topic, PoseStamped, set_goal)
   
   
    rospy.Subscriber(cur_pos_topic,PoseStamped,set_cur_pose)
   
    rospy.Subscriber(cur_vel_topic,TwistStamped,set_cur_vel)
    rospy.Subscriber(check_topic, TwistStamped, check_vel)
    state_sub = rospy.Subscriber('/mavros/state', State, callback=state_cb)
    
    topic_pub = '/mavros/setpoint_velocity/cmd_vel'
    #topic_pub = '/mavros/setpoint_velocity/cmd_vel_unstamped'
    pub = rospy.Publisher(topic_pub, TwistStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    global target_pos 
    global cur_pos
    global cur_vel
    target_pos = PoseStamped()
    cur_pos = PoseStamped()
    cur_vel = TwistStamped()
    vel = TwistStamped()
    vel.header.frame_id = "map"
    global field
    field = [0, 0, 0, 0]
    while not rospy.is_shutdown():
        for i in range(10):
            if i <5 : mode = mode_srv(custom_mode="OFFBOARD") 
            else : pass 
        print(field)
        update_linear_vel(vel,field) 
        #print(u)
        pub.publish(vel)
        rate.sleep()


   
def set_goal(msg):
    global target_pos 
    target_pos = msg
    #print(target_pos)
    
def set_cur_pose(msg):
    global cur_pos
    cur_pos = msg
    # print(cur_pos)
    
def set_cur_vel(msg):
    #print(msg)
    global cur_vel
    cur_vel = msg
    # print(cur_vel)

def check_vel(msg):
    #print(msg)
    pass
def update_fields(msg):
    global field 
    
    field[0] = msg.doubles[0].value
    field[1] = msg.doubles[1].value
    field[2] = msg.doubles[2].value
    field[3] = msg.doubles[3].value
    # print(ki.ints[0].value)
    # print("Kp ={} Kp={} Kp={} ".format(ki,kp,kd))
    # pass


def state_cb(msg):
    # print msg.mode
    if(msg.mode=='OFFBOARD'):
        isReadyToFly = True
        # print "readyToFly"


def trans_q_to_e(obj):
    qx = obj.pose.orientation.x
    qy = obj.pose.orientation.y
    qz = obj.pose.orientation.z
    qw = obj.pose.orientation.w   
    
    rotateZa0 = 2.0*(qx*qy + qw*qz)
    rotateZa1 = qw*qw + qx*qx - qy*qy - qz*qz
    rotateZ = 0.0
    if rotateZa0 != 0.0 and rotateZa1 != 0.0:
        rotateZ = np.arctan2(rotateZa0, rotateZa1)
    return rotateZ

def update_linear_vel(vel,field):
    Kp = 0.7
    Kd = 0.3
    goal_vel = 0.0
    
    vel.twist.linear.x = Kp*(target_pos.pose.position.x-cur_pos.pose.position.x) + Kd*(goal_vel - cur_vel.twist.linear.x) + field[0]
    vel.twist.linear.y = Kp*(target_pos.pose.position.y-cur_pos.pose.position.y) + Kd*(goal_vel - cur_vel.twist.linear.y) + field[1]
    vel.twist.linear.z = Kp*(target_pos.pose.position.z-cur_pos.pose.position.z) + Kd*(goal_vel - cur_vel.twist.linear.z) + field[2]
    
    if vel.twist.linear.x > 2:
        vel.twist.linear.x = 2
    elif vel.twist.linear.x < -2:
        vel.twist.linear.x = -2
    if vel.twist.linear.y > 2:
        vel.twist.linear.y = 2
    elif vel.twist.linear.y < -2:
        vel.twist.linear.y = -2
    if vel.twist.linear.z > 1.5:
        vel.twist.linear.z = 1.5
    elif vel.twist.linear.z < -1.5:
        vel.twist.linear.z = -1.5
    g_z_rot = trans_q_to_e(target_pos)
    c_z_rot = trans_q_to_e(cur_pos)        
    vel.twist.angular.z = Kp*(g_z_rot -c_z_rot) + Kd*(0.0 - cur_vel.twist.angular.z)+ field[3]
    vel.header.stamp = rospy.Time.now()


if __name__ == '__main__':
    sub_and_pub()


