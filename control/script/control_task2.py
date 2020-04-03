#!/usr/bin/env python
import rospy
import numpy as np

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
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
    obstacles = '/obstacles'

    rospy.Subscriber(obstacles,PoseArray, obstr)


    rospy.Subscriber(dynamic_reconfigure,Config, update_fields)
    rospy.Subscriber(goal_topic, PoseStamped, set_goal)
   
   
    rospy.Subscriber(cur_pos_topic,PoseStamped,set_cur_pose)
   
    rospy.Subscriber(cur_vel_topic,TwistStamped,set_cur_vel)
    rospy.Subscriber(check_topic, TwistStamped, check_vel)
    state_sub = rospy.Subscriber('/mavros/state', State, callback=state_cb)
    
    topic_pub = '/mavros/setpoint_velocity/cmd_vel'
    #topic_pub = '/mavros/setpoint_velocity/cmd_vel_unstamped'

    field_node = '/field'
    field = rospy.Publisher(field_node,Twist,queue_size=10)
    
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
    # global cur_x,cur_y,obs_x,obs_y
    field = [0, 0, 0, 0]
    while not rospy.is_shutdown():
        for i in range(10):
            if i <5 : mode = mode_srv(custom_mode="OFFBOARD") 
            else : pass 
        # print(field)
        # 
        update_linear_vel(vel,field) 
        pub.publish(vel)
        
        rate.sleep()
        # nearest_obs(cur_x,cur_y,obs_x,obs_y)
        rep_force(cur_x,cur_y,obs_x,obs_y,field)
    

def obstr(msg):
    global obs_x,obs_y 
    obs_x = [msg.poses[i].position.x for i in range(len(msg.poses))]
    obs_y = [msg.poses[i].position.y for i in range(len(msg.poses))]
    
def nearest_obs(cur_x,cur_y,obs_x,obs_y):
    dist = np.sqrt((obs_x[0]-cur_x)**2+(obs_y[0]-cur_y)**2)
    idx = 0
    for i in range(len(obs_x)):
        d = np.sqrt((obs_x[i]-cur_x)**2+(obs_y[i]-cur_y)**2)
        if d < dist:
            dist = d 
            idx = i
            
    print(dist,"::::",idx )
    return dist, idx

def rep_force(cur_x,cur_y,obs_x,obs_y,field):

    tolerance = 0.7
    k=0.4
    r_field = 0.2
    
    dist_to_obs = np.sqrt((obs_x[0]-cur_x)**2+(obs_y[0]-cur_y)**2)
    idx = 0
    for i in range(len(obs_x)):
        d = np.sqrt((obs_x[i]-cur_x)**2+(obs_y[i]-cur_y)**2)
        if d < dist_to_obs:
            dist_to_obs = d 
            idx = i
                
    # dist_to_obs,idx = nearest_obs(cur_x,cur_y,obs_x,obs_y)

    if (dist_to_obs - r_field) < tolerance:
        field[0] = k*dist_to_obs / (dist_to_obs - r_field)**2 * (cur_x - obs_x[idx])
        field[1] = k*dist_to_obs/ (dist_to_obs - r_field)**2 * (cur_x - obs_y[idx])
    else:
        field[0] = 0
        field[1] = 0
    print(field[0])
 


   
def set_goal(msg):
    global target_pos 
    target_pos = msg
    #print(target_pos)
    
def set_cur_pose(msg):
    global cur_pos,cur_x, cur_y
    cur_pos = msg
    cur_x = msg.pose.position.x
    cur_y = msg.pose.position.y
    
    # print(cur_pos)
    
def set_cur_vel(msg):
    #print(msg)>
    global cur_vel
    cur_vel = msg
    # print(cur_vel)

def check_vel(msg):
    #print(msg)
    pass
def update_fields(msg):
    # global field 
    pass

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
    # print(field[0])
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
