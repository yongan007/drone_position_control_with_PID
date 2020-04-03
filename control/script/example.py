#!/usr/bin/env python
import rospy
import numpy as np

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseArray

from dynamic_reconfigure.server import Server



class PD_controller():
    def __init__(self, kp=0.7, kd=0.3, field=[0,0], tolerance = 0.7, r = 0.2, k = 0.4):
        self.srv = Server(TutorialsConfig, self.callback)
        # topics
        cur_pos_topic = '/mavros/local_position/pose'
        goal_topic = '/goal'
        cur_vel_topic = '/mavros/local_position/velocity_local'
        obstacle_topic = '/obstacles'
        self.target_pos = PoseStamped()
        self.cur_pos = PoseStamped()
        self.cur_vel = TwistStamped()
        self.Kp = kp
        self.Kd = kd
        self.field = field
        self.tolerance = tolerance
        self.obstacles = PoseArray()
        self.r = r
        self.k = k
        rospy.Subscriber(goal_topic, PoseStamped, self.set_goal)
        rospy.Subscriber(cur_pos_topic, PoseStamped, self.set_cur_pose)
        rospy.Subscriber(cur_vel_topic, TwistStamped, self.set_cur_vel)
        rospy.Subscriber(obstacle_topic, PoseArray, self.set_obstacles)
        topic_pub = '/mavros/setpoint_velocity/cmd_vel'
        self.pub = rospy.Publisher(topic_pub, TwistStamped, queue_size=10)

    def callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {Kp}, {Kd}, {Tolerance}, {k}""".format(**config))
        self.Kp = config['Kp']
        self.Kd = config['Kd']
        self.tolerance = config['Tolerance']
        self.k = config['k']
        return config

    def set_goal(self, msg):
        self.target_pos = msg
        # print(target_pos)

    def set_cur_pose(self, msg):
        self.cur_pos = msg
        # print(cur_pos)

    def set_cur_vel(self, msg):
        self.cur_vel = msg
        # print(cur_vel)

    def set_obstacles(self, msg):
        self.obstacles = msg

    def trans_q_to_e(self, obj):
        qx = obj.pose.orientation.x
        qy = obj.pose.orientation.y
        qz = obj.pose.orientation.z
        qw = obj.pose.orientation.w

        rotateZa0 = 2.0 * (qx * qy + qw * qz)
        rotateZa1 = qw * qw + qx * qx - qy * qy - qz * qz
        rotateZ = 0.0
        if rotateZa0 != 0.0 and rotateZa1 != 0.0:
            rotateZ = np.arctan2(rotateZa0, rotateZa1)
        return rotateZ

    def find_near_obstacle(self):
        dist = np.sqrt((self.obstacles.poses[0].position.x - self.cur_pos.pose.position.x)**2 + (self.obstacles.poses[0].position.y - self.cur_pos.pose.position.y)**2)
        closest_ind = 0
        for i in range(len(self.obstacles.poses)):
            temp = np.sqrt((self.obstacles.poses[i].position.x - self.cur_pos.pose.position.x)**2 + (self.obstacles.poses[i].position.y - self.cur_pos.pose.position.y)**2)
            if dist > temp:
                dist = temp
                closest_ind = i
        dist = dist
        return dist, closest_ind

    def potential_field(self):
        distance, obs_ind = self.find_near_obstacle()
        # print(distance)
        if (distance - self.r) < self.tolerance:
            self.field[0] = self.k / (distance - self.r)**2 * (self.cur_pos.pose.position.x - self.obstacles.poses[obs_ind].position.x)/distance
            self.field[1] = self.k / (distance - self.r)**2 * (self.cur_pos.pose.position.y - self.obstacles.poses[obs_ind].position.y)/distance
        else:
            self.field[0] = 0
            self.field[1] = 0

    def update_velocity(self,u):
        goal_vel = 0.0
        u.twist.linear.x = self.Kp * (self.target_pos.pose.position.x - self.cur_pos.pose.position.x) + self.Kd * (
                    goal_vel - self.cur_vel.twist.linear.x) + self.field[0]
        u.twist.linear.y = self.Kp * (self.target_pos.pose.position.y - self.cur_pos.pose.position.y) + self.Kd * (
                    goal_vel - self.cur_vel.twist.linear.y) + self.field[1]
        u.twist.linear.z = self.Kp * (self.target_pos.pose.position.z - self.cur_pos.pose.position.z) + self.Kd * (
                    goal_vel - self.cur_vel.twist.linear.z)
        if u.twist.linear.x > 2:
            u.twist.linear.x = 2
        elif u.twist.linear.x < -2:
            u.twist.linear.x = -2
        if u.twist.linear.y > 2:
            u.twist.linear.y = 2
        elif u.twist.linear.y < -2:
            u.twist.linear.y = -2
        if u.twist.linear.z > 1.5:
            u.twist.linear.z = 1.5
        elif u.twist.linear.z < -1.5:
            u.twist.linear.z = -1.5
        g_z_rot = self.trans_q_to_e(self.target_pos)
        c_z_rot = self.trans_q_to_e(self.cur_pos)
        u.twist.angular.z = self.Kp * (g_z_rot - c_z_rot) + self.Kd * (0.0 - self.cur_vel.twist.angular.z)
        u.header.stamp = rospy.Time.now()

    def start(self):
        rate = rospy.Rate(10)  # 10hz
        u = TwistStamped()
        u.header.frame_id = "map"
        rate.sleep()
        while not rospy.is_shutdown():
            self.potential_field()
            self.update_velocity(u)
            self.pub.publish(u)
            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('control', anonymous=True)
    control = PD_controller()
    control.start()