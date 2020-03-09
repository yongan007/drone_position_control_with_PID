#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from dynamic_tutorials.cfg import TutorialsConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Force}, {distance_to_obstacles},\ 
          {distance_of_tolerance}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_confique", anonymous = False)

    srv = Server(TutorialsConfig, callback)
    rospy.spin()