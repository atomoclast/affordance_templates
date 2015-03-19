#!/usr/bin/env python
import os
import sys
import rospy

from affordance_template_server import AffordanceTemplateServer

if __name__ == '__main__':

    rospy.init_node('AffordanceTemplateServer')

    robot_yaml = rospy.get_param("~robot_config")

    server = AffordanceTemplateServer(robot_yaml)
    server.start()
