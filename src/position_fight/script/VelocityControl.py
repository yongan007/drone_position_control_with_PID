#!/usr/bin/env python

import rospy
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import math
import numpy
from geometry_msgs.msg import TwistStamped
from VelocityController import VelocityController



class QuadController:

    curr_vel = TwistStamped()
    sim_ctr = 1

    des_pose = PoseStamped()
    cur_pose = PoseStamped()

    isReadyToFly = True

    target = Pose()
    target.position.x = 5
    target.position.y = 2
    target.position.z = 3
    


    def __init__(self):

       
        rospy.init_node('Velocity_Controller', anonymous=True)
        vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
       
        vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, callback=self.vel_cb)
        pos_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, callback=self.pos_cb)
        ext_state_sub = rospy.Subscriber('/mavros/extended_state', ExtendedState, callback=self.ext_state_cb)
        state_sub = rospy.Subscriber('/mavros/state', State, callback=self.state_cb)

        rate = rospy.Rate(10)  # Hz
        rate.sleep()

        arming_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        mode = arming_srv(value=True)

                
        vController = VelocityController()
        vController.setTarget(self.target)

        while not rospy.is_shutdown():
            if self.isReadyToFly:
                mode = mode_srv(custom_mode="OFFBOARD")
                self.des_vel = vController.update(self.cur_pose)
                vel_pub.publish(self.des_vel)
            rate.sleep()

    def copy_vel(self, vel):
        copied_vel = TwistStamped()
        copied_vel.header= vel.header
        return copied_vel

    def vel_cb(self, msg):
        # print msg
        self.curr_vel = msg
        print ""

    def pos_cb(self, msg):
        print msg
        self.cur_pose = msg

    def state_cb(self,msg):
        print msg.mode
        if(msg.mode=='OFFBOARD'):
            self.isReadyToFly = True
            print "readyToFly"
            
    def ext_state_cb(self, msg):
        if(msg.landed_state == 1):
            self.landed = True


if __name__ == "__main__":
    QuadController()
