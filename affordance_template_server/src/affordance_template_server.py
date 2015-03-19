import os
import sys
import signal
import yaml
import json
import glob

from threading import Thread
        
import rospy
import roslib; roslib.load_manifest("affordance_template_markers")
import rospkg

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from interactive_markers.interactive_marker_server import *

from affordance_template_markers.robot_interface import *
from affordance_template_markers.affordance_template import *
from affordance_template_markers.template_utilities import *
from affordance_template_markers.affordance_template_data import *
from affordance_template_markers.affordance_template_collection import *

from service_interface import ServiceInterface


class AffordanceTemplateServer(Thread):
    """Affordance Template server."""

    def __init__(self, robot_yaml=""):
        Thread.__init__(self)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        if robot_yaml=="" :
            rospy.logwarn("AffordanceTemplateServer::init() -- NO ROBOT YAML GIVEN, BE SURE TO LOAD ROBOT FROM SERVICE!!")
            
        self.robot_interface = None
        self.structure = {}
      
        self.status = False

        self.server = InteractiveMarkerServer("affordance_template_server")

        # get path to template marker package
        self.library_package_path = self.get_package_path("affordance_template_library")

        # get path to actual template source files
        self.template_path = os.path.join(self.library_package_path, 'templates')
        self.robot_path = os.path.join(self.library_package_path, 'robots')

        # get data from library
        self.robot_map = self.get_available_robots(self.robot_path)
        self.at_data = self.get_available_templates(self.template_path)

        # create the robot interface and configure if there is an input yaml
        if not os.path.isfile(robot_yaml) :
            robot_yaml = self.robot_path + "/" + robot_yaml
            rospy.loginfo(str("AffordanceTemplateServer::init() -- robot_yaml: " + robot_yaml))
            
        if os.path.isfile(robot_yaml) : 
            self.robot_interface = RobotInterface(yaml_file=robot_yaml)
            self.robot_interface.configure()
            rospy.loginfo(str("AffordanceTemplateServer::init() -- robot_yaml: " + robot_yaml + " -- configuring..."))
        else :
            rospy.loginfo(str("AffordanceTemplateServer::init() -- no robot_yaml given"))
            self.robot_interface = RobotInterface()
 
        rospy.loginfo(str("AffordanceTemplateServer::init() -- finished initialization"))
           

    def configure_server(self):
        """Configure the interface connections for clients."""
        self.interfaces = {}
        self.interfaces['service'] = ServiceInterface(self)
        self.status = True
        

    def run(self):
        self.configure_server()
        while not rospy.is_shutdown():
            rospy.sleep(1)

    def get_status(self) :
        return self.status

    def remove_template(self, class_type, instance_id):
        """Stop a template process and remove it from the server's map.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype bool
        @returns True if process was stopped/removed.
        """
        rospy.loginfo(str("AffordanceTemplateServer::remove_template() -- " + class_type + ":" + str(instance_id)))
        if class_type in self.at_data.class_map.keys() :
            if instance_id in self.at_data.class_map[class_type]:
                self.at_data.class_map[class_type][instance_id].terminate()
                del self.at_data.class_map[class_type][instance_id]
        else :
            return False
        return True

    def add_template(self, class_type, instance_id):
        """Start a template process using subprocess.Popen.

        @type class_type string
        @param class_type The class type e.g. "Wheel", "Car", etc.

        @type instance_id int
        @param instance_id The ID of this instance.

        @rtype int
        @returns The Popen object started by the server.
        """
        if class_type in self.at_data.class_map:
            at = AffordanceTemplate(self.server, instance_id, robot_interface=self.robot_interface)
            filename = self.at_data.file_map[class_type]
            at.load_from_file(filename)
            self.at_data.class_map[class_type][instance_id] = at  # TODO this is dumb, need to just have a local list of multiple ATs
            rospy.loginfo(str("AffordanceTemplateServer::add_template() -- adding template " + str(class_type) + " with id: " + str(instance_id)))
            return True
        else :
            rospy.logerr(str("AffordanceTemplateServer::add_template() -- no template type " + str(class_type)))
            return False

    def get_next_template_id(self, class_type):
        ids = self.at_data.class_map[class_type].keys()
        i = 0
        while True:
            if i in ids:
                i += 1
            else:
                return i
        return i

    def get_package_path(self, pkg):
        """Return the path to the ROS package."""
        try:
            rp = rospkg.RosPack()
            return rp.get_path(pkg)
        except:
            rospy.logwarn(str("RobotInterface::AffordanceTemplateServer() -- can't find package: " + str(pkg)))
            return ""

    def get_template_path(self):
        """Return the path to the template nodes."""
        try:
            rp = rospkg.RosPack()
            return os.path.join(rp.get_path('affordance_template_markers'), 'src', 'affordance_template_markers')
        except:
            rospy.logwarn('AffordanceTemplateServer::get_template_path() -- No package found: affordance_template_markers')
            return ""

    def get_available_templates(self, path, input_at_data=None):
        """Parse affordance_templates manifest for available classes."""

        at_data = AffordanceTemplateCollection()

        if input_at_data :
            at_data = input_at_data
        else :
            at_data.class_map = {}
            at_data.traj_map = {}
            at_data.image_map = {}
            at_data.file_map = {}
            at_data.waypoint_map = {}
            at_data.object_map = {}

        os.chdir(path)

        for atfn in glob.glob("*.json") :
            
            try :
                atf = open(atfn).read()
                structure = json.loads(atf)

                at_name = str(structure['name'])
                image = str(structure['image'])
                if not at_name in at_data.class_map :   
                    at_data.class_map[at_name] = {}
                if not at_name in at_data.traj_map :   
                    at_data.traj_map[at_name] = []
                if not at_name in at_data.image_map :   
                    at_data.image_map[at_name] = image
                if not at_name in at_data.file_map :   
                    at_data.file_map[at_name] = os.path.join(path,atfn)

                rospy.loginfo(str("AffordanceTemplateServer() -- found AT File: " + at_name))

                for traj in structure['end_effector_trajectory'] :
                    traj_name = str(traj['name'])
                    at_data.traj_map[at_name].append(traj_name)
                    key = (at_name,traj_name)
                    if not key in at_data.waypoint_map :
                        at_data.waypoint_map[key] = {}
                    
                    for ee_group in traj['end_effector_group'] :
                        ee_id = ee_group['id']
                        wp_id = len(ee_group['end_effector_waypoint'])
                        at_data.waypoint_map[key][ee_id] = wp_id

                at_data.object_map[at_name] = []
                for obj in structure['display_objects'] :
                    obj_name = str(obj['name'])
                    at_data.object_map[at_name].append(obj_name)
                
                rospy.loginfo(str("AffordanceTemplateServer::get_available_templates() -- parsed: " + atfn))

            except :
                rospy.logdebug(str("AffordanceTemplateServer::get_available_templates() -- error parsing " + atfn))

        return at_data

    def load_from_file(self, filename) :
        return json.loads(open(filename).read())

    def get_available_robots(self, path):
        """Parse parses available robots from fs."""

        # robot data storage
        robot_map = {}

        os.chdir(path)
        for r in glob.glob("*.yaml") :
            ri = RobotInterface(yaml_file=r)
           
            # check to see if package actually is "installed" and only add it if it is.
            if self.get_package_path(ri.robot_config.config_package) :
                robot_map[r] = ri
                rospy.loginfo("AffordanceTemplateServer::get_available_robots(" + r + ") - config package found")
            else :
                rospy.logdebug("AffordanceTemplateServer::get_available_robots(" + r + ") - config package not found, not adding")

        return robot_map

