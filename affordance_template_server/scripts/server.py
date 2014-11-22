#!/usr/bin/env python
import os
import sys
import rospy

from affordance_template_server import AffordanceTemplateServer

if __name__ == '__main__':
    # TODO: parge arguments properly
    robot_yaml = ""
    if len(sys.argv) > 1:
        robot_yaml = str(sys.argv[1])

    rospy.init_node('AffordanceTemplateServer')

    server = AffordanceTemplateServer(robot_yaml)
    server.start()
