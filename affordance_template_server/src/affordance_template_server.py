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
from affordance_template_markers.recognition_object import *
from affordance_template_markers.affordance_template_data import *
from affordance_template_markers.affordance_template_collection import *

from service_interface import ServiceInterface


class AffordanceTemplateServer(Thread):
    """Affordance Template server."""

    def __init__(self, robot_yaml=""):
        Thread.__init__(self)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        self.robot_interface = None
        self.structure = {}
      
        self.server = InteractiveMarkerServer("affordance_template_server")

        # get path to template marker package
        self.library_package_path = self.get_package_path("affordance_template_library")

        # get path to actual template source files
        self.template_path = os.path.join(self.library_package_path, 'templates')
        self.robot_path = os.path.join(self.library_package_path, 'robots')
        self.recognition_object_path = os.path.join(self.library_package_path, 'recognition_objects')

        # get data from library
        self.robot_map = self.get_available_robots(self.robot_path)
        self.at_data = self.get_available_templates(self.template_path)
        self.ro_data = self.get_available_recognition_objects(self.recognition_object_path)

        self.running_templates = {}
        self.running_recog_objects = {}

        # create the robot interface and configure if there is an input yaml
        if not os.path.isfile(robot_yaml) :
            robot_yaml = self.robot_path + "/" + robot_yaml
            
        if os.path.isfile(robot_yaml) : 
            self.robot_interface = RobotInterface(yaml_file=robot_yaml)
            self.robot_interface.configure()
        else :
            self.robot_interface = RobotInterface()

    def configure_server(self):
        """Configure the interface connections for clients."""
        # self.recognition_object_update_flags = {}
        self.interfaces = {}
        self.interfaces['service'] = ServiceInterface(self)


    def run(self):
        self.configure_server()
        while not rospy.is_shutdown():
            rospy.sleep(1)

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
        # print "class map keys: ", self.at_data.class_map.keys()
        if class_type in self.at_data.class_map.keys() :
            # print " ids: ", self.at_data.class_map[class_type].keys()
            if instance_id in self.at_data.class_map[class_type]:
                self.at_data.class_map[class_type][instance_id].terminate()
                if self.running_templates[instance_id] == class_type:
                    del self.running_templates[instance_id]
                del self.at_data.class_map[class_type][instance_id]
        else :
            return False
        return True

    # def remove_recognition_object(self, object_type, instance_id):
    #     """Stop a template process and remove it from the server's map.

    #     @type class_type string
    #     @param object_type The class type e.g. "Wheel", "Car", etc.

    #     @type instance_id int
    #     @param instance_id The ID of this instance.

    #     @rtype bool
    #     @returns True if process was stopped/removed.
    #     """
    #     if object_type in self.recognition_object_map and instance_id in self.recognition_object_map[object_type]:
    #         self.recognition_object_map[object_type][instance_id].terminate()
    #         del self.recognition_object_map[object_type][instance_id]

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
            self.running_templates[instance_id] = class_type
            self.at_data.class_map[class_type][instance_id] = at  # TODO this is dumb, need to just have a local list of multiple ATs
            rospy.loginfo(str("AffordanceTemplateServer::add_template() -- adding template " + str(class_type) + " with id: " + str(instance_id)))
            return True
        else :
            rospy.logerr(str("AffordanceTemplateServer::add_template() -- no template type " + str(class_type)))
            return False

    # def start_recognition_process(self, object_type, launch_file, package, topic, instance_id):
    #     """Start a template process using subprocess.Popen.

    #     @type object_type string
    #     @param object_type The class type e.g. "handle", "torus", etc.

    #     @type launch_file string
    #     @param launch_file The launch file to run

    #     @type package string
    #     @param package The package the launch file is in

    #     @rtype int
    #     @returns The Popen object started by the server.
    #     """
    #     if not self.get_package_path(package) :
    #         rospy.loginfo("AffordanceTemplateServer::start_recognition_process(" + object_type + ") No package found: " + package)
    #         return False
    #     # should check if launch file exists as well here

    #     self.running_recog_objects[instance_id] = object_type

    #     # print self.recognition_object_subscribers.keys()
    #     # print self.recognition_object_subscribers[object_type].keys()
    #     self.recognition_object_subscribers[object_type][instance_id] = rospy.Subscriber(topic, MarkerArray, self.recognition_object_callback)

    #     import subprocess
    #     cmd = str('roslaunch ' + package + ' ' + launch_file)
    #     proc = subprocess.Popen([cmd], shell=True)
    #     pid = proc.pid # <--- access `pid` attribute to get the pid of the child process.

    #     self.recognition_object_map[object_type][instance_id] = proc

    #     return True

    def get_next_template_id(self, class_type):
        ids = self.at_data.class_map[class_type].keys()
        i = 0
        while True:
            if i in ids:
                i += 1
            else:
                return i
        return i

    # def get_next_recog_object_id(self, object_type):
    #     ids = self.recognition_object_map[object_type].keys()
    #     i = 0
    #     while True:
    #         if i in ids:
    #             i += 1
    #         else:
    #             return i

    def get_package_path(self, pkg):
        """Return the path to the ROS package."""
        try:
            rp = rospkg.RosPack()
            return rp.get_path(pkg)
        except:
            rospy.logwarn(str('AffordanceTemplateServer::get_package_path() -- No package found: ' + pkg))
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
            except :
                rospy.logwarn(str("AffordanceTemplateServer::get_available_templates() -- error parsing " + atfn))

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
            if self.get_package_path(ri.robot_config.moveit_config_package) :
                robot_map[r] = ri
            else :
                rospy.logwarn("AffordanceTemplateServer::get_available_robots(" + r + ") - MoveIt! config package not found, not adding")

        return robot_map

    def get_available_recognition_objects(self, path):
        """Parse parses available robots from fs."""

        ro_data = RecognitionObjectCollection()
        ro_data.object_map = {}
        ro_data.image_map = {}
        ro_data.launch_map = {}
        ro_data.package_map = {}
        ro_data.marker_topic_map = {}
        
        import glob
        os.chdir(path)
        for r in glob.glob("*.yaml") :
            # print "found recognition_object yaml: ", r
            ro = RecognitionObject()
            ro.load_from_file(r)
            ro_data.object_map[ro.type] = {}
            ro_data.image_map[ro.type] = ro.image_path
            ro_data.launch_map[ro.type] = ro.launch_file
            ro_data.package_map[ro.type] = ro.package
            ro_data.marker_topic_map[ro.type] = ro.topic
                        
        return ro_data #recognition_object_map, recognition_object_info, recognition_object_subscribers


    # def load_recognition_object_from_msg(self, recognition_object) :
    #     r = RecogntionObject()

    #     print "creating new robot from pb message"
    #     try:

    #         r.robot_name = robot.name
    #         r.config_package = robot.moveit_config_package
    #         r.frame_id = robot.frame_id
    #         print "loading robot: " , r.robot_name

    #         r.root_offset.position.x = robot.root_offset.position.x
    #         r.root_offset.position.y = robot.root_offset.position.y
    #         r.root_offset.position.z = robot.root_offset.position.z
    #         r.root_offset.orientation.x = robot.root_offset.orientation.x
    #         r.root_offset.orientation.y = robot.root_offset.orientation.y
    #         r.root_offset.orientation.z = robot.root_offset.orientation.z
    #         r.root_offset.orientation.w = robot.root_offset.orientation.w

    #         r.end_effector_names = []
    #         r.end_effector_name_map = {}
    #         r.manipulator_id_map = {}
    #         r.manipulator_pose_map = {}
    #         r.tool_offset_map = {}

    #         for ee in robot.end_effectors.end_effector:
    #             r.end_effector_names.append(ee.name)
    #             r.end_effector_name_map[ee.id] = ee.name
    #             r.manipulator_id_map[ee.name] = ee.id
                
    #             p = geometry_msgs.msg.Pose()
    #             p.position.x = ee.pose_offset.position.x
    #             p.position.y = ee.pose_offset.position.y
    #             p.position.z = ee.pose_offset.position.z
    #             p.orientation.x = ee.pose_offset.orientation.x
    #             p.orientation.y = ee.pose_offset.orientation.y
    #             p.orientation.z = ee.pose_offset.orientation.z
    #             p.orientation.w = ee.pose_offset.orientation.w
    #             r.manipulator_pose_map[ee.name] = p

    #             t = geometry_msgs.msg.Pose()
    #             t.position.x = ee.tool_offset.position.x
    #             t.position.y = ee.tool_offset.position.y
    #             t.position.z = ee.tool_offset.position.z
    #             t.orientation.x = ee.tool_offset.orientation.x
    #             t.orientation.y = ee.tool_offset.orientation.y
    #             t.orientation.z = ee.tool_offset.orientation.z
    #             t.orientation.w = ee.tool_offset.orientation.w
    #             r.tool_offset_map[ee.name] = t

    #         print "done!"
    #         return r

    #     except :
    #         rospy.logerr("AffordanceTemplateServer::load_recog_object_from_msg() -- error parsing robot protobuf file")
    #         return None

    # def recognition_object_callback(self, data) :
    #     for m in data.markers :
    #         id = m.ns + str(m.id)
    #         if not id in self.recognition_object_update_flags :
    #             print "creating a new AT for a recognized object"
    #             roat = AffordanceTemplate(self.server, m.id, m.ns, robot_interface=self.robot_interface)
    #             roat.load_from_marker(m)
    #             self.recognition_object_update_flags[id] = False
