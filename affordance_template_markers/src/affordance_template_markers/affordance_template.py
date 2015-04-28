import yaml
import json

import os
import shutil      
import random
import time
        
import copy
import PyKDL as kdl
import tf
import threading

import rospy
import rospkg
import roslib; roslib.load_manifest("affordance_template_markers")

import geometry_msgs.msg
import sensor_msgs.msg
import visualization_msgs.msg

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *

from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.models.urdf_helper import *

from affordance_template_markers.robot_interface import *
from affordance_template_markers.frame_store import *
from affordance_template_markers.template_utilities import *
from affordance_template_markers.affordance_template_data import *

class AffordanceTemplate(threading.Thread) :

    def __init__(self, server, id, name="affordance_template", initial_pose=None, robot_interface=None):
        super(AffordanceTemplate,self).__init__()
        
        self.mutex = threading.Lock()
        self._stop = threading.Event()

        self.menu_handler = MenuHandler()
        self.menu_handler.insert("Delete", callback=self.delete_callback)
        self.server = server
        self.frame_id = "world"
        self.id = int(id)
        self.key = name + ":" + str(self.id)
        self.name = name
        self.filename = ""
        self.root_object = ""
        self.parent_map = {}
        self.marker_map = {}
        self.callback_map = {}
        self.marker_pose_offset = {}
        self.display_objects = []
        self.frame_store_map = {}
        self.current_trajectory = ""

        self.object_scale_factor = {}
        self.end_effector_scale_factor = {}

        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_frame = "template:0"

        self.waypoints = {}
        self.structure = None
        self.robot_interface = None

        # parameter storage
        self.object_origin = {}
        self.object_controls = {}
        self.object_geometry = {}
        self.object_material = {}

        self.waypoint_origin = {}
        self.waypoint_controls = {}
        self.waypoint_end_effectors = {}
        self.waypoint_ids = {}              # holds ids for actual end effector movement
        self.path_plan_ids = {}             # holds ids for planned end effector movement
        self.waypoint_pose_map = {}

        # menu stuff
        self.marker_menus = {}
        self.menu_handles = {}

        # control helper stuff
        self.waypoint_index = {}
        self.waypoint_plan_index = {}            
        self.waypoint_backwards_flag = {}
        self.waypoint_auto_execute = {}
        self.waypoint_plan_valid = {}
        self.waypoint_execution_valid = {}
        self.waypoint_loop = {}
        self.waypoint_max = {}
        self.waypoint_controls_display_on = {} 
        self.object_controls_display_on = True
        # helper frames
        self.robotTroot = kdl.Frame()
        self.rootTobj = {}
        self.objTwp = {}
        self.wpTee = {}
        self.eeTtf = {}

        # not all templates will have a dynamic reconfigure server
        self.dserver = None
        self.initial_pose = initial_pose

        self.random = random.Random()
        self.random.seed(time.clock)

        if not isinstance(robot_interface, RobotInterface) :
            rospy.loginfo("AffordanceTemplate::init() -- problem setting robot config")
        # else :
        #     print "FIX THIS"
        #     print robot_interface
        self.robot_interface = robot_interface

        # set up menu info
        self.waypoint_menu_options = []
        # self.waypoint_menu_options.append(("Display Next Path Segment", False))
        # self.waypoint_menu_options.append(("Display Full Path", False))
        # self.waypoint_menu_options.append(("Compute Backwards Path", True))
        # self.waypoint_menu_options.append(("Execute Next Segment", False))
        # self.waypoint_menu_options.append(("Execute Full Path", False))
        # self.waypoint_menu_options.append(("Loop Path", True))
        # self.waypoint_menu_options.append(("Sync To Actual", False))
        # self.waypoint_menu_options.append(("Manipulator Stored Poses", False))
        # self.waypoint_menu_options.append(("End-Effector Stored Poses", False))
        self.waypoint_menu_options.append(("Change End-Effector Pose", False))
        self.waypoint_menu_options.append(("Hide Controls", True))
        self.waypoint_menu_options.append(("Add Waypoint Before", False))
        self.waypoint_menu_options.append(("Add Waypoint After", False))
        self.waypoint_menu_options.append(("Delete Waypoint", False))
        self.waypoint_menu_options.append(("Move Forward", False))
        self.waypoint_menu_options.append(("Move Back", False))

        self.object_menu_options = []
        # self.object_menu_options.append(("Display Next Path Segment", False))
        # self.object_menu_options.append(("Display Full Path", False))
        # self.object_menu_options.append(("Compute Backwards Path", True))
        # self.object_menu_options.append(("Execute Next Segment", False))
        # self.object_menu_options.append(("Execute Full Path", False))
        self.object_menu_options.append(("Add Waypoint Before", False))
        self.object_menu_options.append(("Add Waypoint After", False))
        self.object_menu_options.append(("Reset", False))
        self.object_menu_options.append(("Save", False))
        self.object_menu_options.append(("Hide Controls", True))
        self.object_menu_options.append(("Choose Trajectory", False))

        # start the frame update thread
        self.running = True
        self.start()

        rospy.loginfo("AffordanceTemplate::init() -- Done Creating new Empty AffordanceTemplate")

    def get_package_path(self, pkg):
        """Return the path to the ROS package."""
        try:
            rp = rospkg.RosPack()
            return rp.get_path(pkg)
        except:
            # print 'No package found: ', pkg
            return False

    def set_root_object(self, name) :
        self.root_object = name

    def get_root_object(self) :
        return self.root_object

    def create_waypoint_id(self, ee_id, wp_id) :
        return str(str(ee_id) + "." + str(wp_id) + ":" + str(self.name))

    def add_interactive_marker(self, marker, callback=None):
        name = marker.name
        rospy.logdebug(str("Adding affordance template marker: " + name))
        self.marker_map[name] = marker
        if callback:
            self.callback_map[name] = callback
            return self.server.insert(marker, callback)
        else:
            self.callback_map[name] = self.process_feedback
            return self.server.insert(marker, self.process_feedback)

    def remove_interactive_marker(self, marker_name):
        if self.server.erase(marker_name):
            if marker_name in self.marker_map:
                del self.marker_map[marker_name]
                del self.callback_map[marker_name]
                return True
        return False

    def remove_all_markers(self) :
        markers = copy.deepcopy(self.marker_map)
        for m in markers :
            self.remove_interactive_marker(m)

    def attach_menu_handler(self, marker):
        return self.menu_handler.apply(self.server, marker.name)

    def get_marker(self):
        return self.server.get(self._key)

    def has_marker(self):
        if self._key in self.marker_map.keys():
            return True
        else:
            return False

    def delete_callback(self, feedback):
        for key in self.marker_map.keys():
            self.server.erase(key)
        self.server.applyChanges()
        # self.tear_down()

    def tear_down(self, keep_alive=False):
        if self.dserver:
            self.dserver.set_service.shutdown("User deleted template.")
        if not keep_alive:
            rospy.signal_shutdown("User deleted template.")

    def append_id(self, s) :
        return str(s + ":" + str(self.id))

    def is_parent(self, child, parent) :
        if not child in self.parent_map :
            return False
        elif self.parent_map[child] == None :
            return False
        elif self.parent_map[child] == parent:
            return True
        else :
            return self.is_parent(self.parent_map[child], parent)

    def get_chain(self, parent, child) :
        if not self.is_parent(child, parent) :
            return kdl.Frame()
        else :
            if parent == self.parent_map[child] :
                T = self.rootTobj[child]
            else :
                T = self.get_chain(parent,self.parent_map[child])*self.rootTobj[child]
            return T

    def get_chain_from_robot(self, child) :
        return self.get_chain("robot",child)

    def pose_from_origin(self, origin) :
        p = geometry_msgs.msg.Pose()
        p.orientation.w = 1
        try:
            q = (kdl.Rotation.RPY(origin['rpy'][0],origin['rpy'][1],origin['rpy'][2])).GetQuaternion()
            p.position.x = origin['xyz'][0]
            p.position.y = origin['xyz'][1]
            p.position.z = origin['xyz'][2]
            p.orientation.x = q[0]
            p.orientation.y = q[1]
            p.orientation.z = q[2]
            p.orientation.w = q[3]
        except :
            rospy.logerr("AffordanceTemplate::pose_from_origin() error")
        return p

    def load_initial_parameters(self) :

        rospy.loginfo("AffordanceTemplate::load_initial_parameters()")
        # can we just make a deep copy of the json structure here?

        frames = self.frame_store_map.keys()
        for k in frames: 
            del self.frame_store_map[k]
        self.frame_store_map = {}

        if self.robot_interface == None :
            rospy.error("AffordanceTemplate::load_initial_parameters() -- no robot config")
            return False

        if self.structure == None :
            rospy.error("AffordanceTemplate::load_initial_parameters() -- no structure")
            return False

        self.display_objects = []
        self.object_scale_factor = {}
        self.end_effector_scale_factor = {}
        self.waypoints = {}

        self.frame_id = self.robot_interface.robot_config.frame_id
        self.robotTroot = getFrameFromPose(self.robot_interface.robot_config.root_offset)

        self.name = self.structure['name']
        self.key = self.name

        # parse objects
        ids = 0
        for obj in self.structure['display_objects'] :

            obj_name = self.name + "/" + obj['name']

            rospy.loginfo(str("AffordanceTemplate::load_initial_parameters() -- adding object: " + obj_name))

            self.display_objects.append(obj_name)
            self.object_scale_factor[obj_name] = 1.0
            self.end_effector_scale_factor[obj_name] = 1.0

            # first object should have no parent, make it "robot" by default
            if ids == 0:
                self.set_root_object(obj_name)
                self.parent_map[obj_name] = "robot"
            else :
                try :
                    self.parent_map[obj_name] = self.name + "/" + obj['parent']
                except :
                    pass

            # store object info to local structs
            self.rootTobj[obj_name] = getFrameFromPose(self.pose_from_origin(obj['origin']))
            self.marker_pose_offset[obj_name] = self.pose_from_origin(obj['origin'])
            self.object_origin[obj_name] = obj['origin']
            self.object_controls[obj_name] = obj['controls']
            self.object_geometry[obj_name] = obj['shape']
            
            if not obj['shape']['type'] == 'mesh' :
                self.object_material[obj_name] = obj['shape']['material']
            
            ids += 1

        for traj in self.structure['end_effector_trajectory'] :

            # check if all the end effectors in the trajectory actually exist                       
            if not self.is_valid_trajectory(traj) :
                rospy.logwarn(str("AffordanceTemplate::load_initial_parameters() -- not adding trajectory: " + traj['name']))
                continue

            # all ee's were found, can actually create things
            rospy.loginfo(str("AffordanceTemplate::load_initial_parameters() -- adding trajectory: " + traj['name']))
            self.create_trajectory_structures(traj['name'])             

            for ee_group in traj['end_effector_group'] :
                wp_id = 0
                for wp in ee_group['end_effector_waypoint'] :

                    # parse end effector information
                    wp_name = self.append_id(str(ee_group['id']) + "." + str(wp_id))                   
                    ee_name = self.robot_interface.end_effector_name_map[ee_group['id']]

                    wp_parent = wp['display_object'] 
                    wp_parent_new = self.name + "/" + wp['display_object'] 
                    
                    if not wp_parent_new in self.display_objects :
                        rospy.logerr(str("AffordanceTemplate::load_initial_parameters() -- end-effector display object " + wp_parent_new + " not found!"))

                    if not wp_parent_new == None :
                        parent = self.name + "/" + wp['display_object']
                    else :
                        parent = self.get_root_object()

                    if wp['ee_pose'] == None :
                        pose_id = None
                    else :
                        pose_id = wp['ee_pose']
                    wp_pose = self.pose_from_origin(wp['origin'])

                    # create waypoint we can play with
                    self.create_waypoint(traj['name'], ee_group['id'], wp_id, wp_pose, parent, wp['controls'], wp['origin'], pose_id)

                    wp_id += 1

 
    def is_valid_trajectory(self, traj) : 
        valid_trajectory = True             
        try :
            for ee_group in traj['end_effector_group'] :
                for wp in ee_group['end_effector_waypoint'] :
                    if not ee_group['id'] in self.robot_interface.end_effector_name_map :
                        rospy.logdebug(str("AffordanceTemplate::is_valid_trajectory() -- can't find ee: " + ee_group['id']))
                        valid_trajectory = False     
        except :
            rospy.logerr("AffordanceTemplate::is_valid_trajectory() -- malformed trajectory structure")
            valid_trajectory = False        

        return valid_trajectory


    def create_from_parameters(self, keep_poses=False, current_trajectory="") :

        rospy.loginfo("AffordanceTemplate::create_from_parameters()")
        self.key = self.name
        self.frame_store_map[self.name] = FrameStore(self.key, self.robot_interface.robot_config.frame_id, getPoseFromFrame(self.robotTroot))
        if not current_trajectory :
            for traj in self.structure['end_effector_trajectory'] :
                if self.is_valid_trajectory(traj) :
                    self.current_trajectory = str(traj['name'])
                    rospy.loginfo("AffordanceTemplate::create_from_parameters() -- setting current trajectory to: " + str(traj['name']))
                    break
        
        # parse objects
        ids = 0
        debug_id = 0

        for obj in self.display_objects :

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            self.setup_object_menu(obj)

            root_frame = self.name
            obj_frame = obj

            if self.get_root_object() == obj :
                root_frame = self.key
            else :
                if obj in self.parent_map :
                     root_frame = self.parent_map[obj]

            int_marker.header.frame_id = str("/" + root_frame)
            int_marker.name = obj
            int_marker.description = obj
            int_marker.scale = self.object_controls[obj]['scale']*self.object_scale_factor[obj]

            control = InteractiveMarkerControl()
            control.interaction_mode = InteractiveMarkerControl.BUTTON

            marker = Marker()
            marker.ns = obj
            marker.id = ids
                
            if not keep_poses or not obj in self.frame_store_map.keys() :
                self.frame_store_map[obj] = FrameStore(obj_frame, root_frame, copy.deepcopy(self.marker_pose_offset[obj]))
                int_marker.pose = copy.deepcopy(self.marker_pose_offset[obj])
            else :
                int_marker.pose = copy.deepcopy(self.frame_store_map[obj].pose)
                # THIS IS A HACK, NOT SURE WHY ITS NECESSARY
                parent_obj = self.parent_map[obj]
                if parent_obj in self.object_scale_factor :
                    int_marker.pose.position.x /= self.object_scale_factor[parent_obj]
                    int_marker.pose.position.y /= self.object_scale_factor[parent_obj]
                    int_marker.pose.position.z /= self.object_scale_factor[parent_obj]


            # adjust for scale factor of parent object
            parent_obj = self.parent_map[obj]
            if parent_obj in self.object_scale_factor :
                int_marker.pose.position.x *= self.object_scale_factor[parent_obj]
                int_marker.pose.position.y *= self.object_scale_factor[parent_obj]
                int_marker.pose.position.z *= self.object_scale_factor[parent_obj]

                self.frame_store_map[obj].pose = int_marker.pose


            if self.object_geometry[obj]['type'] == "mesh" :
                marker.type = Marker.MESH_RESOURCE
                marker.mesh_resource = self.object_geometry[obj]['data']
                marker.mesh_use_embedded_materials = True
                marker.scale.x = self.object_geometry[obj]['size'][0]*self.object_scale_factor[obj]
                marker.scale.y = self.object_geometry[obj]['size'][1]*self.object_scale_factor[obj]
                marker.scale.z = self.object_geometry[obj]['size'][2]*self.object_scale_factor[obj]
                # print " --- drawing mesh: ", obj, " with scale: ", self.object_scale_factor[obj]
            elif self.object_geometry[obj]['type'] == "box" :
                marker.type = Marker.CUBE
                marker.scale.x = self.object_geometry[obj]['size'][0]*self.object_scale_factor[obj]
                marker.scale.y = self.object_geometry[obj]['size'][1]*self.object_scale_factor[obj]
                marker.scale.z = self.object_geometry[obj]['size'][2]*self.object_scale_factor[obj]
            elif self.object_geometry[obj]['type'] == "sphere" :
                marker.type = Marker.SPHERE
                marker.scale.x = self.object_geometry[obj]['size'][0]*self.object_scale_factor[obj]
                marker.scale.y = self.object_geometry[obj]['size'][1]*self.object_scale_factor[obj]
                marker.scale.z = self.object_geometry[obj]['size'][2]*self.object_scale_factor[obj]
            elif self.object_geometry[obj]['type'] == "cylinder" :
                marker.type = Marker.CYLINDER
                marker.scale.x = self.object_geometry[obj]['radius']*self.object_scale_factor[obj]
                marker.scale.y = self.object_geometry[obj]['length']*self.object_scale_factor[obj]
 
            control.markers.append(marker)

            if self.object_geometry[obj]['type'] != "mesh" :
                control.markers[0].color.r = self.object_material[obj]['rgba'][0]
                control.markers[0].color.g = self.object_material[obj]['rgba'][1]
                control.markers[0].color.b = self.object_material[obj]['rgba'][2]
                control.markers[0].color.a = self.object_material[obj]['rgba'][3]
            else :
                control.markers[0].mesh_use_embedded_materials = False


            scale = 1.0
            if obj in self.object_controls :
                scale = self.object_controls[obj]['scale']*self.object_scale_factor[obj]

             # int_marker = CreateInteractiveMarker(self.frame_id, obj.name, scale)
            int_marker.controls.append(control)
            if(self.object_controls_display_on) :
                int_marker.controls.extend(CreateCustomDOFControls("",
                    self.object_controls[obj]['xyz'][0], self.object_controls[obj]['xyz'][1], self.object_controls[obj]['xyz'][2],
                    self.object_controls[obj]['rpy'][0], self.object_controls[obj]['rpy'][1], self.object_controls[obj]['rpy'][2]))


            self.marker_map[obj] = control.markers[0]
            self.marker_pose_offset[obj] = self.pose_from_origin(self.object_origin[obj])

            if self.object_controls_display_on :
                self.marker_menus[obj].setCheckState( self.menu_handles[(obj,"Hide Controls")], MenuHandler.UNCHECKED )
            else :
                self.marker_menus[obj].setCheckState( self.menu_handles[(obj,"Hide Controls")], MenuHandler.CHECKED )

            self.add_interactive_marker(int_marker)
            self.marker_menus[obj].apply( self.server, obj )
            self.server.applyChanges()

            ids += 1

            if self.current_trajectory :
                self.create_trajectory_from_parameters(self.current_trajectory)
            else :
                rospy.logwarn("AffordanceTemplate::create_from_parameters() -- couldn't find a valid trajectory to set as the current")

        rospy.loginfo("AffordanceTemplate::create_from_parameters() -- done")


    # parse end effector trajectory information
    def create_trajectory_from_parameters(self, trajectory) :

        rospy.loginfo("AffordanceTemplate::create_trajectory_from_parameters()")

        self.current_trajectory = trajectory

        wp_ids = 0
        for wp in self.waypoints[self.current_trajectory] :

            ee_name = self.robot_interface.end_effector_name_map[int(self.waypoint_end_effectors[trajectory][wp])]
            
            root_frame = self.name #str(self.name + ":" + str(self.id))
            if (trajectory, wp) in self.parent_map :
                root_frame = self.parent_map[(trajectory,wp)] #str(self.parent_map[wp] + ":" + str(self.id))

                if not self.parent_map[(trajectory,wp)] in self.display_objects :
                    rospy.logerr(str("AffordanceTemplate::create_from_parameters() -- end-effector display object " + str(self.parent_map[(trajectory,wp)]) + "not found!"))

            display_pose = getPoseFromFrame(self.objTwp[trajectory][wp])

            # adjust for scale factor of parent object
            parent_obj = self.parent_map[(trajectory,wp)]
            parent_scale = self.object_scale_factor[parent_obj]*self.end_effector_scale_factor[parent_obj]
            # mag = math.sqrt(display_pose.position.x**2 + display_pose.position.y**2 + display_pose.position.z**2)
            # scaled_mag = parent_scale*mag 
            display_pose.position.x *= parent_scale
            display_pose.position.y *= parent_scale
            display_pose.position.z *= parent_scale

            int_marker = InteractiveMarker()
            control = InteractiveMarkerControl()

            int_marker.header.frame_id = str("/" + root_frame)
            int_marker.pose = display_pose
            int_marker.name = wp
            int_marker.description = wp
            int_marker.scale = self.waypoint_controls[trajectory][wp]['scale']

            menu_control = InteractiveMarkerControl()
            menu_control.interaction_mode = InteractiveMarkerControl.BUTTON

            # IS THIS A PROBLEM WITH TRAJECTORY NOT BEING ACCOUNTED FOR?
            self.marker_menus[wp] = MenuHandler()
            self.setup_waypoint_menu(wp, ee_name)

            # set default menu options for the waypoint. This is ugly, but i blame how the IM menus work...
            id = int(self.waypoint_end_effectors[trajectory][wp])
            if self.waypoint_backwards_flag[trajectory][id] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Compute Backwards Path")], MenuHandler.CHECKED )
            if self.waypoint_auto_execute[trajectory][id] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Execute On Move")], MenuHandler.CHECKED )
            if self.waypoint_loop[trajectory][id] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Loop Path")], MenuHandler.CHECKED )
            
            if self.waypoint_controls_display_on[trajectory][wp] :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Hide Controls")], MenuHandler.UNCHECKED )
            else :
                self.marker_menus[wp].setCheckState( self.menu_handles[(wp,"Hide Controls")], MenuHandler.CHECKED )

            # ask for end_effector pose markers here (and change ids/colors as necessary)
            id = self.waypoint_pose_map[trajectory][wp]
            if id == None :
                pn = "current"
            else :
                pn = self.robot_interface.end_effector_id_map[ee_name][id]
                
            markers = self.robot_interface.end_effector_link_data[ee_name].get_markers_for_pose(pn)

            for m in markers.markers :

                try :
                    ee_m = copy.deepcopy(m)                   
                    ee_m.header.frame_id = ""
                    ee_m.ns = self.name
                    ee_m.pose = getPoseFromFrame(self.wpTee[wp]*self.eeTtf[wp]*getFrameFromPose(m.pose))
                    menu_control.markers.append( ee_m )
                except :
                    rospy.logdebug("passing on marker")

            scale = 1.0
            if wp in self.waypoint_controls[trajectory] :
                scale = self.waypoint_controls[trajectory][wp]['scale']
            int_marker.controls.append(menu_control)
            if(self.waypoint_controls_display_on[trajectory][wp]) :
                int_marker.controls.extend(CreateCustomDOFControls("",
                    self.waypoint_controls[trajectory][wp]['xyz'][0], self.waypoint_controls[trajectory][wp]['xyz'][1], self.waypoint_controls[trajectory][wp]['xyz'][2],
                    self.waypoint_controls[trajectory][wp]['rpy'][0], self.waypoint_controls[trajectory][wp]['rpy'][1], self.waypoint_controls[trajectory][wp]['rpy'][2]))

            self.add_interactive_marker(int_marker)
            self.marker_menus[wp].apply( self.server, wp )
            self.server.applyChanges()

            wp_ids += 1

            rospy.loginfo("AffordanceTemplate::create_trajectory_from_parameters() -- done")

    def update_template_defaults(self, objects=True, waypoints=True) :

        if objects :
            for obj in self.object_origin.keys() :
                current_pose = self.frame_store_map[obj].pose
                xyz = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
                rpy = (kdl.Rotation.Quaternion(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)).GetRPY()
                for i in range(3) :
                    self.object_origin[obj]['xyz'][i] = xyz[i]
                    self.object_origin[obj]['rpy'][i] = rpy[i]

        if waypoints :
            for traj in self.waypoints.keys() :
                for wp in self.waypoints[traj] :
                    current_pose = getPoseFromFrame(self.objTwp[traj][wp])
                    xyz = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
                    rpy = (kdl.Rotation.Quaternion(current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z,current_pose.orientation.w)).GetRPY()
                    for i in range(3) :
                        self.waypoint_origin[traj][wp]['xyz'][i] = xyz[i]
                        self.waypoint_origin[traj][wp]['rpy'][i] = rpy[i]


    def save_to_disk(self, filename=None, package=None, image="", new_class_type="",save_scaling=False) :

        class_type = self.structure['name'].split(":")[0]

        if filename == None :
            filename = class_type + str(".json")

        filename_bak = filename + str(".bak.") + str(self.random.randint(0,100000))
        if package == None:
            package = "affordance_template_library"

        # get package and file path
        package_path = self.get_package_path(package)
        template_path    = os.path.join(package_path, 'templates')
        image_path    = "package://" + package + "/images/"
        output_file = os.path.join(template_path, filename)

        rospy.loginfo(("Writing template JSON to file: " + output_file))

        # backup file
        try :
            dstname = os.path.join(template_path, filename_bak)
            shutil.copyfile(output_file,dstname)
        except :
            rospy.loginfo("AffordanceTemplate::save_to_disk() -- no file to backup")

        # set current object and waypoint positions to "default"
        self.update_template_defaults()

        new_structure = {}

        if new_class_type :
            new_class_type = new_class_type.split(":")[0]
            new_structure['name'] = new_class_type
        else :
            new_structure['name'] = class_type

        if image :
            image = os.path.join(image_path, image)
            new_structure['image'] = image
            self.structure['image'] = image
        else :
            new_structure['image'] = self.structure['image']

        new_structure['display_objects'] = []
        
        obj_id = 0
        for obj in self.display_objects :
            key = obj.split("/")[1].split(":")[0]

            try :
                parent_obj = self.parent_map[obj]  
            except :
                parent_obj = "robot"

            new_structure['display_objects'].append({})
            new_structure['display_objects'][obj_id]['name'] = key
                       
            if save_scaling :
                self.object_controls[obj]['scale'] *= self.object_scale_factor[obj]
                if self.object_geometry[obj]['type'] == "mesh" or self.object_geometry[obj]['type'] == "box" or self.object_geometry[obj]['type'] == "sphere" :
                    self.object_geometry[obj]['size'][0] *= self.object_scale_factor[obj]
                    self.object_geometry[obj]['size'][1] *= self.object_scale_factor[obj]
                    self.object_geometry[obj]['size'][2] *= self.object_scale_factor[obj]
                elif self.object_geometry[obj]['type'] == "cylinder" :
                    self.object_geometry[obj]['radius'] *= self.object_scale_factor[obj]
                    self.object_geometry[obj]['length'] *= self.object_scale_factor[obj]

                # adjust for scale factor of parent object
                if parent_obj in self.object_scale_factor :
                    parent_scale = self.object_scale_factor[parent_obj]
                    # mag = math.sqrt( self.object_origin[obj]['xyz'][0]**2 + self.object_origin[obj]['xyz'][1]**2 + self.object_origin[obj]['xyz'][2]**2)
                    # scaled_mag = parent_scale*mag 
                    self.object_origin[obj]['xyz'][0] *= parent_scale
                    self.object_origin[obj]['xyz'][1] *= parent_scale
                    self.object_origin[obj]['xyz'][2] *= parent_scale
            else :
                rospy.logdebug("not saving scaling for objects")

            new_structure['display_objects'][obj_id]['controls'] = self.object_controls[obj]
            new_structure['display_objects'][obj_id]['shape'] = self.object_geometry[obj]
            new_structure['display_objects'][obj_id]['origin'] = self.object_origin[obj]

            if parent_obj != "robot" :
                try :
                    new_structure['display_objects'][obj_id]['parent'] = parent_obj.split("/")[1].split(":")[0]
                except :
                    rospy.logwarn(str("AffordanceTemplate::save_to_disk() malformed parent: " + parent_obj))
           
            obj_id += 1

        traj_id = 0
        new_structure['end_effector_trajectory'] = []
            
        for traj in self.structure['end_effector_trajectory'] :
            traj_name = traj['name']
            new_structure['end_effector_trajectory'].append({})
            new_structure['end_effector_trajectory'][traj_id]['name'] = traj['name']
            new_structure['end_effector_trajectory'][traj_id]['end_effector_group'] = []

            idx = 0

            # for ee in self.structure['end_effector_trajectory'][traj_id]['end_effector_group'] :
            for ee_id in self.waypoint_index[traj_name].keys() :               
                new_structure['end_effector_trajectory'][traj_id]['end_effector_group'].append({})
                new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['id'] = int(ee_id)
                new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['end_effector_waypoint'] = []
            
                wp_id = 0
                for wp in self.waypoints[traj['name']] :
                    if int(wp.split(".")[0]) == int(ee_id) :
                        parent_key = self.parent_map[(traj['name'],wp)].split("/")[1].split(":")[0]
              
                        new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['end_effector_waypoint'].append({})
                        new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['end_effector_waypoint'][wp_id]['ee_pose'] = self.waypoint_pose_map[traj['name']][wp]
                        new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['end_effector_waypoint'][wp_id]['display_object'] = parent_key
                        new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['end_effector_waypoint'][wp_id]['controls'] = self.waypoint_controls[traj['name']][wp]

                        if save_scaling :
                            display_pose = Pose()
                            display_pose.position.x = self.waypoint_origin[traj['name']][wp]['xyz'][0]
                            display_pose.position.y = self.waypoint_origin[traj['name']][wp]['xyz'][1]
                            display_pose.position.z = self.waypoint_origin[traj['name']][wp]['xyz'][2]

                            parent_obj = self.parent_map[(traj['name'],wp)]
                            parent_scale = self.object_scale_factor[parent_obj]*self.end_effector_scale_factor[parent_obj]
                            
                            # mag = math.sqrt(display_pose.position.x**2 + display_pose.position.y**2 + display_pose.position.z**2)
                            # scaled_mag = parent_scale*mag 
                           
                            self.waypoint_origin[traj['name']][wp]['xyz'][0] = display_pose.position.x*parent_scale
                            self.waypoint_origin[traj['name']][wp]['xyz'][1] = display_pose.position.y*parent_scale
                            self.waypoint_origin[traj['name']][wp]['xyz'][2] = display_pose.position.z*parent_scale
                        else :
                            rospy.logdebug("not saving scaling for end-effectors")

                        new_structure['end_effector_trajectory'][traj_id]['end_effector_group'][idx]['end_effector_waypoint'][wp_id]['origin'] = self.waypoint_origin[traj['name']][wp]

                        wp_id += 1
                idx += 1
            traj_id += 1
    
        try :
            with open(str(output_file), 'w') as outfile :
                rospy.loginfo(str("AffordanceTemplate::save_to_disk() -- writing out new JSON file: " + filename))
                json.dump(new_structure, outfile, sort_keys = False, indent = 2)
        except :
            rospy.logerr(str("AffordanceTemplate::save_to_disk() -- problem writing out JSON file: " + filename))
            return False
        return True


    def create_trajectory_structures(self, traj_name) :
        if not traj_name in self.waypoints : self.waypoints[traj_name] = []
        if not traj_name in self.objTwp : self.objTwp[traj_name] = {}
        if not traj_name in self.waypoint_controls : self.waypoint_controls[traj_name] = {}
        if not traj_name in self.waypoint_origin : self.waypoint_origin[traj_name] = {}
        if not traj_name in self.waypoint_end_effectors : self.waypoint_end_effectors[traj_name] = {}
        if not traj_name in self.waypoint_ids : self.waypoint_ids[traj_name] = {}
        if not traj_name in self.path_plan_ids : self.path_plan_ids[traj_name] = {}
        if not traj_name in self.waypoint_pose_map : self.waypoint_pose_map[traj_name] = {}
        if not traj_name in self.waypoint_max : self.waypoint_max[traj_name] = {}
        if not traj_name in self.waypoint_index : self.waypoint_index[traj_name] = {}
        if not traj_name in self.waypoint_backwards_flag : self.waypoint_backwards_flag[traj_name] = {}
        if not traj_name in self.waypoint_auto_execute : self.waypoint_auto_execute[traj_name] = {}
        if not traj_name in self.waypoint_plan_valid : self.waypoint_plan_valid[traj_name] = {}
        if not traj_name in self.waypoint_execution_valid : self.waypoint_execution_valid[traj_name] = {}        
        if not traj_name in self.waypoint_loop : self.waypoint_loop[traj_name] = {}
        if not traj_name in self.waypoint_controls_display_on : self.waypoint_controls_display_on[traj_name] = {}
        if not traj_name in self.waypoint_plan_index : self.waypoint_plan_index[traj_name] = {}


    def create_waypoint(self, traj_name, ee_id, wp_id, ps, parent, controls=None, origin=None, pose_id=None) :

        wp_name = self.create_waypoint_id(ee_id, wp_id)
        ee_name = self.robot_interface.end_effector_name_map[ee_id]

        rospy.loginfo(str("Creating waypoint: " + wp_name))
        self.parent_map[(traj_name,wp_name)] = parent

        self.objTwp[traj_name][wp_name] = getFrameFromPose(ps)

        ee_offset = self.robot_interface.manipulator_pose_map[ee_name]
        tool_offset = self.robot_interface.tool_offset_map[ee_name]

        self.wpTee[wp_name] = getFrameFromPose(ee_offset)
        self.eeTtf[wp_name] = getFrameFromPose(tool_offset)

        if controls == None : controls = self.create_6dof_controls(0.25)
        self.waypoint_controls[traj_name][wp_name] = controls

        if origin == None : origin = self.create_origin_from_pose(ps)
        self.waypoint_origin[traj_name][wp_name] = origin

        self.waypoint_end_effectors[traj_name][wp_name] = ee_id
        self.waypoint_ids[traj_name][wp_name] = wp_id
        rospy.loginfo(str("Waypoint id at " + traj_name + " with name " + wp_name + "is " + str(wp_id)))
        self.path_plan_ids[traj_name][wp_name] = wp_id          # for using to plan multiple waypoints before executing

        self.waypoint_pose_map[traj_name][wp_name] = pose_id

        self.waypoint_controls_display_on[traj_name][wp_name] = False

        if ee_id not in self.waypoint_max[traj_name]: self.waypoint_max[traj_name][ee_id] = 0
        if self.waypoint_end_effectors[traj_name][wp_name] not in self.waypoint_index[traj_name] :
            self.waypoint_index[traj_name][ee_id] = -1
            self.waypoint_plan_index[traj_name][ee_id] = -1
            self.waypoint_backwards_flag[traj_name][ee_id] = False
            self.waypoint_auto_execute[traj_name][ee_id] = False
            self.waypoint_plan_valid[traj_name][ee_id] = False
            self.waypoint_execution_valid[traj_name][ee_id] = False
            self.waypoint_loop[traj_name][ee_id] = False
        else :
            if int(self.waypoint_ids[traj_name][wp_name]) > self.waypoint_max[traj_name][ee_id] :
                self.waypoint_max[traj_name][ee_id] = int(self.waypoint_ids[traj_name][wp_name])

        rospy.loginfo(str("Waypoint index at " + traj_name + " with ee " + str(ee_id) + " is " + str(self.waypoint_index[traj_name][ee_id])))

        if not wp_name in self.waypoints[traj_name] :
            self.waypoints[traj_name].append(wp_name)
            self.waypoints[traj_name].sort()

    def remove_waypoint(self, traj_name, ee_id, wp_id) :
        max_idx = self.waypoint_max[traj_name][ee_id]
        for k in range(wp_id, max_idx) : self.move_waypoint(traj_name, ee_id, k+1, k)
        last_wp_name = self.create_waypoint_id(ee_id, self.waypoint_max[traj_name][ee_id])
        
        for k in self.parent_map.keys() :
            if k == (traj_name,last_wp_name) :
                del self.parent_map[k]
        del self.objTwp[traj_name][last_wp_name]
        del self.wpTee[last_wp_name]
        del self.eeTtf[last_wp_name]
        del self.waypoint_controls[traj_name][last_wp_name]
        del self.waypoint_origin[traj_name][last_wp_name]
        del self.waypoint_end_effectors[traj_name][last_wp_name]
        del self.waypoint_ids[traj_name][last_wp_name]
        del self.path_plan_ids[traj_name][last_wp_name]
        del self.waypoint_pose_map[traj_name][last_wp_name]
        del self.waypoint_controls_display_on[traj_name][last_wp_name]
        self.waypoint_max[traj_name][ee_id] -= 1
        while last_wp_name in self.waypoints[traj_name]: self.waypoints[traj_name].remove(last_wp_name)
        self.waypoints[traj_name].sort()
        self.remove_interactive_marker(last_wp_name)

    def move_waypoint(self, traj_name, ee_id, old_id, new_id) :
        old_name = self.create_waypoint_id(ee_id, old_id)
        new_name = self.create_waypoint_id(ee_id, new_id)
        self.create_waypoint(traj_name, ee_id, new_id, getPoseFromFrame(self.objTwp[traj_name][old_name]), self.parent_map[(traj_name,old_name)], 
            copy.deepcopy(self.waypoint_controls[traj_name][old_name]), copy.deepcopy(self.waypoint_origin[traj_name][old_name]), copy.deepcopy(self.waypoint_pose_map[traj_name][old_name]))

    def swap_waypoints(self, traj_name, ee_id, wp_id1, wp_id2) :
        wp_name1 = self.create_waypoint_id(ee_id, wp_id1)
        wp_name2 = self.create_waypoint_id(ee_id, wp_id2)

        objTwp1 = self.objTwp[traj_name][wp_name1]
        objTwp2 = self.objTwp[traj_name][wp_name2]

        parent1 = self.parent_map[(traj_name,wp_name1)]
        parent2 = self.parent_map[(traj_name,wp_name2)]

        pose_map1 = self.waypoint_pose_map[traj_name][wp_name1]
        pose_map2 = self.waypoint_pose_map[traj_name][wp_name2]

        controls1 = self.waypoint_controls[traj_name][wp_name1]
        controls2 = self.waypoint_controls[traj_name][wp_name2]

        origin1 = self.waypoint_origin[traj_name][wp_name1]
        origin2 = self.waypoint_origin[traj_name][wp_name2]

        self.create_waypoint(traj_name, ee_id, wp_id1, getPoseFromFrame(objTwp2), parent2, controls2, origin2, pose_map2)
        self.create_waypoint(traj_name, ee_id, wp_id2, getPoseFromFrame(objTwp1), parent1, controls1, origin1, pose_map1)

    def create_from_structure(self) :
        rospy.loginfo("AffordanceTemplate::create_from_structure() -- loading initial parameters")
        self.load_initial_parameters()
        rospy.loginfo("AffordanceTemplate::create_from_structure() -- creating from parameters")
        self.create_from_parameters()
        rospy.loginfo("AffordanceTemplate::create_from_structure() -- done")

    def setup_object_menu(self, obj) :
        self.marker_menus[obj] = MenuHandler()
        for m,c in self.object_menu_options :
            if m == "Add Waypoint Before" or m == "Add Waypoint After":
                sub_menu_handle = self.marker_menus[obj].insert(m)
                for ee in self.robot_interface.end_effector_name_map.iterkeys() :
                    name = self.robot_interface.end_effector_name_map[ee]
                    self.menu_handles[(obj,m,name)] = self.marker_menus[obj].insert(name,parent=sub_menu_handle,callback=self.create_waypoint_callback)
            elif m == "Choose Trajectory" :
                sub_menu_handle = self.marker_menus[obj].insert(m)
                for traj in self.structure['end_effector_trajectory'] :
                    traj_name = str(traj['name'])
                    self.menu_handles[(obj,m,traj_name)] = self.marker_menus[obj].insert(traj_name,parent=sub_menu_handle,callback=self.trajectory_callback)
                    if traj_name == self.current_trajectory :
                        self.marker_menus[obj].setCheckState( self.menu_handles[(obj,m,traj_name)], MenuHandler.CHECKED )
                    else :
                        self.marker_menus[obj].setCheckState( self.menu_handles[(obj,m,traj_name)], MenuHandler.UNCHECKED )                       
            else :
                self.menu_handles[(obj,m)] = self.marker_menus[obj].insert( m, callback=self.process_feedback )
                if c : self.marker_menus[obj].setCheckState( self.menu_handles[(obj,m)], MenuHandler.UNCHECKED )

    def add_trajectory_to_object_menus(self, traj_name) :   
        self.structure['end_effector_trajectory'].append({})
        n = len(self.structure['end_effector_trajectory'])-1
        self.structure['end_effector_trajectory'][n]['name'] = traj_name
        for obj in self.display_objects :
            self.setup_object_menu(obj)    
            self.marker_menus[obj].apply( self.server, obj )
            self.server.applyChanges()

    def add_trajectory(self, traj_name) :
        self.create_trajectory_structures(traj_name)
        self.add_trajectory_to_object_menus(traj_name)

    def scale_object(self, object_name, scale_factor, end_effector_adjustment=1.0) :
        obj_name = self.name + "/" + object_name + ":" + str(self.id)
        self.object_scale_factor[obj_name] = scale_factor
        self.end_effector_scale_factor[obj_name] = end_effector_adjustment
        self.remove_all_markers()
        self.create_from_parameters(True, self.current_trajectory)

    def setup_waypoint_menu(self, waypoint, group) :
        for m,c in self.waypoint_menu_options :
            if m == "Manipulator Stored Poses" :
                sub_menu_handle = self.marker_menus[waypoint].insert(m)
                parent_group = self.robot_interface.path_planner.get_srdf_model().get_end_effector_parent_group(group)
                for p in self.robot_interface.path_planner.get_stored_state_list(parent_group) :
                    self.menu_handles[(waypoint,m,p)] = self.marker_menus[waypoint].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            elif m == "End-Effector Stored Poses" :
                sub_menu_handle = self.marker_menus[waypoint].insert(m)
                for p in self.robot_interface.path_planner.get_stored_state_list(group) :
                    self.menu_handles[(waypoint,m,p)] = self.marker_menus[waypoint].insert(p,parent=sub_menu_handle,callback=self.stored_pose_callback)
            elif m == "Change End-Effector Pose" :
                sub_menu_handle = self.marker_menus[waypoint].insert(m)
                for p in self.robot_interface.path_planner.get_stored_state_list(group) :
                    self.menu_handles[(waypoint,m,p)] = self.marker_menus[waypoint].insert(p,parent=sub_menu_handle,callback=self.change_ee_pose_callback)
            else :
                self.menu_handles[(waypoint,m)] = self.marker_menus[waypoint].insert( m, callback=self.process_feedback )
                if c : self.marker_menus[waypoint].setCheckState( self.menu_handles[(waypoint,m)], MenuHandler.UNCHECKED )

    def load_from_file(self, filename) :
        atf = open(filename).read()
        self.structure = json.loads(atf)
        self.structure = self.append_id_to_structure(self.structure)
        self.load_initial_parameters()
        self.create_from_parameters()
        stuff = filename.split("/")
        self.filename = stuff[len(stuff)-1]
        return self.structure

    def load_from_marker(self, m) :
        self.load_initial_parameters_from_marker(m)
        self.create_from_parameters()
        return self.structure

    def append_id_to_structure(self, structure) :
        structure['name'] = self.append_id(str(structure['name']))
        for obj in structure['display_objects'] :
            obj['name'] = self.append_id(obj['name'])
            try :
                obj['parent'] = self.append_id(obj['parent'])
            except :
                rospy.logwarn(str("AffordanceTemplate::append_id_to_structure() -- no parent for " + obj['name']))
        for traj in structure['end_effector_trajectory'] :
            for ee_group in traj['end_effector_group'] :
                for wp in ee_group['end_effector_waypoint'] :
                    wp['display_object'] = self.append_id(wp['display_object'])
                    
                    
        return structure

    def print_structure(self) :
        print self.structure
        
    def process_feedback(self, feedback):

        if feedback.marker_name in self.display_objects :
            self.frame_store_map[feedback.marker_name].pose = feedback.pose

        if feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP :

            if feedback.marker_name in self.waypoints[self.current_trajectory] :
                self.objTwp[self.current_trajectory][feedback.marker_name] = getFrameFromPose(feedback.pose)
                
        elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:

            handle = feedback.menu_entry_id

            if feedback.marker_name in self.display_objects :
                ee_list = self.waypoint_index[self.current_trajectory].keys()
                if handle == self.menu_handles[(feedback.marker_name,"Reset")] :
                    rospy.loginfo(str("AffordanceTemplate::process_feedback() -- Reseting Affordance Template"))
                    self.create_from_structure()

                if handle == self.menu_handles[(feedback.marker_name,"Save")] :
                    rospy.loginfo(str("AffordanceTemplate::process_feedback() -- Saving Affordance Template"))
                    self.save_to_disk(self.filename)

                if handle == self.menu_handles[(feedback.marker_name,"Hide Controls")] :
                    state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                    if state == MenuHandler.CHECKED:
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                        self.object_controls_display_on = True
                    else :
                        self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                        self.object_controls_display_on = False

                    # this is (also) wrong
                    self.remove_all_markers()
                    self.create_from_parameters(True, self.current_trajectory)

                    rospy.loginfo(str("AffordanceTemplate::process_feedback() -- setting Hide Controls flag to " + str(self.object_controls_display_on)))

            else :
                ee_list =[int(feedback.marker_name.split(".")[0])]
                waypoint_id = int(feedback.marker_name.split(".")[1].split(":")[0])

            for ee_id in ee_list :

                ee_name = self.robot_interface.get_end_effector_name(ee_id)
                manipulator_name = self.robot_interface.get_manipulator(ee_name)
                ee_offset = self.robot_interface.manipulator_pose_map[ee_name]
                tool_offset = self.robot_interface.tool_offset_map[ee_name]
                max_idx = self.waypoint_max[self.current_trajectory][ee_id]

                next_path_idx = self.compute_next_path_id(ee_id, 1, self.waypoint_backwards_flag[self.current_trajectory][ee_id])
 
                if handle == self.menu_handles[(feedback.marker_name,"Add Waypoint Before")] :
                    rospy.loginfo(str("Adding Waypoint before " + str(waypoint_id) + "for end effector: " + str(ee_id)))

                    new_pose = geometry_msgs.msg.Pose()
                    first_name = self.create_waypoint_id(ee_id,waypoint_id)
                    pose_first = getPoseFromFrame(self.objTwp[self.current_trajectory][first_name])
                    new_id = 0
                    if waypoint_id > 0 :
                        new_id = waypoint_id-1
                        second_name = self.create_waypoint_id(ee_id,new_id)
                        pose_second = getPoseFromFrame(self.objTwp[self.current_trajectory][second_name])
                        new_pose.position.x = (pose_second.position.x - pose_first.position.x)/2.0 + pose_first.position.x
                        new_pose.position.y = (pose_second.position.y - pose_first.position.y)/2.0 + pose_first.position.y
                        new_pose.position.z = (pose_second.position.z - pose_first.position.z)/2.0 + pose_first.position.z
                        new_pose.orientation = copy.deepcopy(pose_first.orientation)
                    else :
                        new_pose = copy.deepcopy(pose_first)
                        new_pose.position.x +=0.025
                        new_pose.position.y +=0.025
                        new_pose.position.z +=0.025

                    r = range(new_id,max_idx+1)
                    r.reverse()
                    for k in r:
                        old_name = self.create_waypoint_id(ee_id, str(k))
                        new_name = self.create_waypoint_id(ee_id, str(k+1))
                        self.move_waypoint(self.current_trajectory, ee_id, k, k+1)

                    # print "creating waypoint at : ", new_id
                    old_name = self.create_waypoint_id(ee_id, str(1))
                    self.create_waypoint(self.current_trajectory, ee_id, waypoint_id, new_pose, self.parent_map[(self.current_trajectory, old_name)], copy.deepcopy(self.waypoint_controls[self.current_trajectory][old_name]), 
                        copy.deepcopy(self.waypoint_origin[self.current_trajectory][old_name]), copy.deepcopy(self.waypoint_pose_map[self.current_trajectory][old_name]))
                    self.create_from_parameters(True, self.current_trajectory)


                if handle == self.menu_handles[(feedback.marker_name,"Add Waypoint After")] :
                    rospy.loginfo(str("Adding Waypoint after " + str(waypoint_id) + "for end effector: " + str(ee_id)))

                    new_pose = geometry_msgs.msg.Pose()
                    first_name = self.create_waypoint_id(ee_id,waypoint_id)
                    pose_first = getPoseFromFrame(self.objTwp[self.current_trajectory][first_name])
                    new_id = waypoint_id+1
                    if waypoint_id < max_idx :
                        second_name = self.create_waypoint_id(ee_id, new_id)
                        pose_second = getPoseFromFrame(self.objTwp[self.current_trajectory][second_name])
                        new_pose.position.x = (pose_second.position.x - pose_first.position.x)/2.0 + pose_first.position.x
                        new_pose.position.y = (pose_second.position.y - pose_first.position.y)/2.0 + pose_first.position.y
                        new_pose.position.z = (pose_second.position.z - pose_first.position.z)/2.0 + pose_first.position.z
                        new_pose.orientation = copy.deepcopy(pose_first.orientation)
                    else :
                        new_pose = copy.deepcopy(pose_first)
                        new_pose.position.x +=0.025
                        new_pose.position.y +=0.025
                        new_pose.position.z +=0.025

                    r = range(waypoint_id,max_idx+1)
                    r.reverse()
                    for k in r:
                        old_name = self.create_waypoint_id(ee_id, str(k))
                        new_name = self.create_waypoint_id(ee_id, str(k+1))
                        self.move_waypoint(self.current_trajectory, ee_id, k, k+1)

                    old_name = self.create_waypoint_id(ee_id, max_idx) 
                    self.create_waypoint(self.current_trajectory, ee_id, new_id, new_pose, self.parent_map[(self.current_trajectory, old_name)], copy.deepcopy(self.waypoint_controls[self.current_trajectory][old_name]), 
                        copy.deepcopy(self.waypoint_origin[self.current_trajectory][old_name]), copy.deepcopy(self.waypoint_pose_map[self.current_trajectory][old_name]))
                    self.create_from_parameters(True, self.current_trajectory)

                if handle == self.menu_handles[(feedback.marker_name,"Delete Waypoint")] :
                    rospy.loginfo(str("Deleting Waypoint " + str(waypoint_id) + "for end effector: " + str(ee_id)))
                    self.remove_waypoint(self.current_trajectory, ee_id, waypoint_id)
                    self.create_from_parameters(True, self.current_trajectory)

                if handle == self.menu_handles[(feedback.marker_name,"Move Forward")] :
                    if waypoint_id < max_idx :
                        self.swap_waypoints(self.current_trajectory, ee_id, waypoint_id, waypoint_id+1)
                        self.create_from_parameters(True, self.current_trajectory)

                if handle == self.menu_handles[(feedback.marker_name,"Move Back")] :
                    if waypoint_id > 0:
                        self.swap_waypoints(self.current_trajectory, ee_id, waypoint_id-1, waypoint_id)
                        self.create_from_parameters(True, self.current_trajectory)

                # waypoint specific menu options
                if not feedback.marker_name in self.display_objects :

                    if handle == self.menu_handles[(feedback.marker_name,"Hide Controls")] :
                        state = self.marker_menus[feedback.marker_name].getCheckState( handle )
                        if state == MenuHandler.CHECKED:
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.UNCHECKED )
                            self.waypoint_controls_display_on[self.current_trajectory][feedback.marker_name] = True
                        else :
                            self.marker_menus[feedback.marker_name].setCheckState( handle, MenuHandler.CHECKED )
                            self.waypoint_controls_display_on[self.current_trajectory][feedback.marker_name] = False

                        # this is wrong
                        self.remove_all_markers()
                        self.create_from_parameters(True, self.current_trajectory)

                        rospy.loginfo(str("AffordanceTemplate::process_feedback() -- setting Hide Controls flag to " + str(self.waypoint_controls_display_on[self.current_trajectory][feedback.marker_name])))

            self.marker_menus[feedback.marker_name].reApply( self.server )

        self.server.applyChanges()

    def trajectory_callback(self, feedback) :

        old_key = (feedback.marker_name,"Choose Trajectory", self.current_trajectory)
        self.marker_menus[feedback.marker_name].setCheckState( self.menu_handles[old_key], MenuHandler.UNCHECKED )

        for traj in self.structure['end_effector_trajectory'] :
            traj_name = str(traj['name'])
            key = (feedback.marker_name,"Choose Trajectory", traj_name)

            if self.menu_handles[key] == feedback.menu_entry_id :
                self.marker_menus[feedback.marker_name].setCheckState( self.menu_handles[key], MenuHandler.CHECKED )

                # clear old trajectory 
                for wp in self.waypoints[self.current_trajectory] :
                    ee_id = self.waypoint_end_effectors[self.current_trajectory][wp]
                    wp_id = self.waypoint_ids[self.current_trajectory][wp]
                    self.remove_interactive_marker(wp)

                self.server.applyChanges()

                self.current_trajectory = traj_name

                # re-draw new one
                self.create_trajectory_from_parameters(self.current_trajectory)
        
        self.marker_menus[feedback.marker_name].apply( self.server, feedback.marker_name )
        self.server.applyChanges()

    def set_trajectory(self, traj_name) :

        marker_keys = self.marker_menus.keys()
        for k in marker_keys :
            old_key = (k,"Choose Trajectory", self.current_trajectory)
            new_key = (k,"Choose Trajectory", traj_name)

            if (old_key in self.menu_handles.keys()) and (new_key in self.menu_handles.keys()) :
                self.marker_menus[k].setCheckState( self.menu_handles[old_key], MenuHandler.UNCHECKED )
                self.marker_menus[k].setCheckState( self.menu_handles[new_key], MenuHandler.CHECKED )

                self.marker_menus[k].apply( self.server, k )        

        # clear old trajectory 
        for wp in self.waypoints[self.current_trajectory] :
            ee_id = self.waypoint_end_effectors[self.current_trajectory][wp]
            wp_id = self.waypoint_ids[self.current_trajectory][wp]
            self.remove_interactive_marker(wp)

        self.server.applyChanges()
        self.current_trajectory = traj_name

        # re-draw new one
        # print "creating new one from params"
        self.create_trajectory_from_parameters(self.current_trajectory)
            
        # print "applying changes (again)"
        self.server.applyChanges()



    def stored_pose_callback(self, feedback) :
        ee_id =int(feedback.marker_name.split(".")[0])
        ee_name = self.robot_interface.get_end_effector_name(ee_id)
        manipulator_name = self.robot_interface.get_manipulator(ee_name)
        for p in self.robot_interface.path_planner.get_stored_state_list(ee_name) :
            if self.menu_handles[(feedback.marker_name,"End-Effector Stored Poses",p)] == feedback.menu_entry_id :
                self.robot_interface.path_planner.create_joint_plan_to_target(ee_name, self.robot_interface.stored_poses[ee_name][p])
                r = self.robot_interface.path_planner.execute_plan(ee_name)
                if not r : rospy.logerr(str("RobotTeleop::process_feedback(pose) -- failed execution for group: " + ee_name + ". re-synching..."))
        for p in self.robot_interface.path_planner.get_stored_state_list(manipulator_name) :
            if self.menu_handles[(feedback.marker_name,"Manipulator Stored Poses",p)] == feedback.menu_entry_id :
                self.robot_interface.path_planner.plan_to_joint_goal(manipulator_name, self.robot_interface.stored_poses[manipulator_name][p])
                r = self.robot_interface.path_planner.execute_plan(manipulator_name, from_stored=True, wait=False)
                if not r : rospy.logerr(str("RobotTeleop::process_feedback(pose) -- failed execution for group: " + manipulator_name + ". re-synching..."))


    def change_ee_pose_callback(self, feedback) :
        ee_id =int(feedback.marker_name.split(".")[0])
        ee_name = self.robot_interface.get_end_effector_name(ee_id)
        wp = feedback.marker_name
        pn = None
        for p in self.robot_interface.path_planner.get_stored_state_list(ee_name) :
            if self.menu_handles[(feedback.marker_name,"Change End-Effector Pose",p)] == feedback.menu_entry_id :
                pn = p
                break
        pid = self.robot_interface.end_effector_pose_map[ee_name][pn]
        rospy.loginfo(str("AffordanceTemplate::change_ee_pose_callback() -- setting Waypoint " + str(wp) + " pose to " + str(pn) + " (" + str(pid) + ")"))
        self.waypoint_pose_map[self.current_trajectory][wp] = pid
        self.create_from_parameters(True, self.current_trajectory)


    def create_waypoint_callback(self, feedback) :

        for ee_id in self.robot_interface.end_effector_name_map.iterkeys() :
            ee_name = self.robot_interface.end_effector_name_map[ee_id]
            if self.menu_handles[(feedback.marker_name,"Add Waypoint After",ee_name)] == feedback.menu_entry_id :
                rospy.loginfo(str("AffordanceTemplate::create_waypoint_callback()  -- Add waypoint after: " + feedback.marker_name))
                wp_id = 0
                ps = geometry_msgs.msg.Pose()
                pose_id = None
                if not ee_id in self.waypoint_max[self.current_trajectory].keys() :
                    wp_name = self.create_waypoint_id(ee_id, wp_id)
                    T = getFrameFromPose(feedback.pose)
                    ps = getPoseFromFrame(T)
                    ps.position.x = 0.05
                    ps.position.y = 0.05
                    ps.position.z = 0.05
                    self.create_waypoint(self.current_trajectory, ee_id, wp_id, ps, feedback.marker_name, pose_id)
                else :
                    wp_id = self.waypoint_max[self.current_trajectory][ee_id]+1
                    wp_name = self.create_waypoint_id(ee_id, wp_id)
                    last_wp_name = self.create_waypoint_id(ee_id, self.waypoint_max[self.current_trajectory][ee_id])
                    ps = getPoseFromFrame(self.objTwp[self.current_trajectory][last_wp_name])
                    ps.position.x +=0.025
                    ps.position.y +=0.025
                    ps.position.z +=0.025
                    pose_id = self.waypoint_pose_map[self.current_trajectory][last_wp_name]
                    self.create_waypoint(self.current_trajectory, ee_id, wp_id, ps, feedback.marker_name, self.waypoint_controls[self.current_trajectory][last_wp_name], 
                        self.waypoint_origin[self.current_trajectory][last_wp_name], pose_id)
                self.create_from_parameters(True, self.current_trajectory)

            if self.menu_handles[(feedback.marker_name,"Add Waypoint Before",ee_name)] == feedback.menu_entry_id :
                rospy.loginfo(str("AffordanceTemplate::create_waypoint_callback()  -- Add waypoint before: " + feedback.marker_name))
                wp_id = 0
                wp_name = self.create_waypoint_id(ee_id, str(0))
                ps = geometry_msgs.msg.Pose()
                pose_id = None
                if not ee_id in self.waypoint_max[self.current_trajectory].keys() :
                    ps = feedback.pose
                    ps.position.x = 0.05
                    ps.position.y = 0.05
                    ps.position.z = 0.05
                    self.create_waypoint(self.current_trajectory, ee_id, wp_id, ps, feedback.marker_name, pose_id)
                else :
                    ps = getPoseFromFrame(self.objTwp[self.current_trajectory][wp_name])
                    ps.position.x -=0.025
                    ps.position.y -=0.025
                    ps.position.z -=0.025
                    pose_id = self.waypoint_pose_map[self.current_trajectory][wp_name]
                    r = range(0,self.waypoint_max[self.current_trajectory][ee_id]+1)
                    r.reverse()
                    for k in r:
                        old_name = self.create_waypoint_id(ee_id, str(k))
                        new_name = self.create_waypoint_id(ee_id, str(k+1))
                        self.move_waypoint(self.current_trajectory, ee_id, k, k+1)
                    self.create_waypoint(self.current_trajectory, ee_id, wp_id, ps, feedback.marker_name, copy.deepcopy(self.waypoint_controls[self.current_trajectory][wp_name]), 
                        copy.deepcopy(self.waypoint_origin[self.current_trajectory][wp_name]), pose_id)

                self.create_from_parameters(True, self.current_trajectory)

    # returns an array of waypoint IDs from the current waypoint index to the waypoint "steps" away, and the idea of the final waypoint 
    def compute_path_ids(self, id, steps, backwards=False) :
        idx  = self.waypoint_index[self.current_trajectory][id]
        if steps == 0: 
            return [idx], idx

        max_idx = self.waypoint_max[self.current_trajectory][id]
        path = []
        if steps == 0: return path, idx
        cap = max_idx+1
        inc = 1
        if backwards :
            inc = -1
            if idx < 0: path.append(max_idx)
        for s in range(steps):
            idx += inc
            path.append(idx % cap)

        return path, path[len(path)-1]

    def stop(self, end_effector) :
        manipulator_name = self.robot_interface.get_manipulator(end_effector)
        self.robot_interface.path_planner.groups[manipulator_name].stop()

    def trajectory_has_ee(self, traj_name, ee_name) :

        if not ee_name in self.robot_interface.manipulator_id_map.keys() :
            return False

        ee_id = self.robot_interface.manipulator_id_map[ee_name]
        
        if not traj_name in self.waypoint_max.keys() :
            return False  
        
        if not ee_id in self.waypoint_max[traj_name].keys() :  
            return False

        return True

    def compute_next_path_id(self, id, steps, backwards=False) :

        next_path_idx = -1
        if not id in self.waypoint_index :
            return next_path_idx

        max_idx = self.waypoint_max[self.current_trajectory][id]
        if self.waypoint_index[self.current_trajectory][id] < 0 :
            # haven't started yet, so set first waypoint to 0
            next_path_idx = 0
        else :
            if backwards :
                next_path_idx = self.waypoint_index[self.current_trajectory][id]-steps
                if self.waypoint_loop[self.current_trajectory][id] :
                    if next_path_idx < 0 :
                        next_path_idx = max_idx + next_path_idx
                else :
                    if next_path_idx < 0 :
                        next_path_idx = 0
            else :
                next_path_idx = self.waypoint_index[self.current_trajectory][id]+steps
                if self.waypoint_loop[self.current_trajectory][id] :
                    if  next_path_idx > max_idx :
                        next_path_idx = (self.waypoint_index[self.current_trajectory][id]+steps)-max_idx
                else :
                    if  next_path_idx > max_idx :
                        next_path_idx = max_idx

        return next_path_idx


    def plan_path_to_waypoints(self, end_effectors, steps=1, direct=False, backwards=False) :
    
        rospy.loginfo("AffordanceTemplate::plan_path_to_waypoints()")

        manipulator_names = []
        frame_ids = []
        waypoints_list = []
        ee_ids = []
        next_path_idxs = []
        next_path_strs = []

        ret = {}

        for end_effector in end_effectors:
            ret[end_effector] = False

        for end_effector in end_effectors:

            rospy.loginfo(str("AffordanceTemplate::plan_path_to_waypoints() -- configuring plan goal for " + end_effector))

            ee_id = self.robot_interface.manipulator_id_map[end_effector]
            ee_offset = self.robot_interface.manipulator_pose_map[end_effector]
            tool_offset = self.robot_interface.tool_offset_map[end_effector]
            max_idx = self.waypoint_max[self.current_trajectory][ee_id]
            manipulator_name = self.robot_interface.get_manipulator(end_effector)
            ee_name = self.robot_interface.get_end_effector_name(ee_id)
            self.waypoint_plan_valid[self.current_trajectory][ee_id] = False
            self.waypoint_execution_valid[self.current_trajectory][ee_id] = False
               
            path, next_path_idx = self.compute_path_ids(ee_id, steps, backwards)
            self.waypoint_plan_index[self.current_trajectory][ee_id] = next_path_idx

            if(direct) : path = [next_path_idx]
        
            if next_path_idx == -1 :
                rospy.logerr(str("AffordanceTemplate::plan_path_to_waypoints() -- bad path index for " + end_effector))
                return ret

            next_path_str = self.create_waypoint_id(ee_id, next_path_idx)      
            rospy.loginfo(str("AffordanceTemplate::plan_path_to_waypoints() -- computing path to index[" + str(next_path_str) + "]"))

            waypoints = []
            frame_id = ""

            # print "next_path_idx: ", next_path_idx
            # print path

            try :
                for idx in path :
                    next_path_str = self.create_waypoint_id(ee_id, idx)
                    # print "next_path_str: ", next_path_str
                    if not next_path_str in self.objTwp[self.current_trajectory] :
                        rospy.logerr(str("AffordanceTemplate::plan_path_to_waypoints() -- path index[" + str(next_path_str) + "] not found!!"))
                        return ret
                    else :
                        rospy.loginfo(str("AffordanceTemplate::plan_path_to_waypoints() -- computing path to index[" + str(next_path_str) + "]"))
                        
                        k = str(next_path_str)
                        pt = geometry_msgs.msg.PoseStamped()
                        pt.header = self.server.get(k).header
                        pt.header.stamp = rospy.Time(0)
                        pt.pose = self.server.get(k).pose
                        frame_id =  pt.header.frame_id
                        T_goal = getFrameFromPose(pt.pose)
                        T_offset = getFrameFromPose(ee_offset)
                        T_fixed_joint_offset = self.get_fixed_joint_offset(end_effector).Inverse()
                        T_tool = getFrameFromPose(tool_offset).Inverse()
                        
                        manipulator_group = self.robot_interface.path_planner.get_srdf_model().group_end_effectors[end_effector].parent_group
                        control_frame = self.robot_interface.path_planner.get_control_frame(manipulator_group)
                        end_effector_frame = self.robot_interface.path_planner.get_srdf_model().group_end_effectors[end_effector].parent_link
                        self.tf_listener.waitForTransform(control_frame, end_effector_frame, rospy.Time(0), rospy.Duration(5.0))
                        (trans, rot) = self.tf_listener.lookupTransform(control_frame, end_effector_frame, rospy.Time(0))
                        T_ee = fromMsg(toPose(trans,rot)).Inverse()
                        T_hand = T_goal*T_offset*T_fixed_joint_offset.Inverse()
                        T = T_hand*T_ee

                        pt.pose = getPoseFromFrame(T)
                        waypoints.append(pt)
                        
                   
                    if not self.waypoint_pose_map[self.current_trajectory][next_path_str] == None :
                        rospy.logwarn(str("AffordanceTemplate::plan_path_to_waypoints() -- planning for end_effector: " + ee_name))

                        id = self.waypoint_pose_map[self.current_trajectory][next_path_str]
                        pn = self.robot_interface.end_effector_id_map[ee_name][id]
                        rospy.loginfo("AffordanceTemplate::plan_path_to_waypoints() -- planning path for ee[" + str(ee_name) + "] to " + pn)
                        self.robot_interface.path_planner.create_joint_plan([ee_name], [self.robot_interface.stored_poses[ee_name][pn]])
                        rospy.logwarn(str("AffordanceTemplate::plan_path_to_waypoints() -- planning for end_effector " + ee_name + " done"))

            except :
                rospy.logerr(str("AffordanceTemplate::plan_path_to_waypoints() -- Error in calculating waypoint pose goals from id path"))
                
            
            # create plan through the waypoints 
            # frame_ids.append(frame_id)
            manipulator_names.append(manipulator_name)
            waypoints_list.append(waypoints)
            ee_ids.append(ee_id)
            next_path_strs.append(next_path_str)
            next_path_idxs.append(next_path_idx)

        try :
            planner_result = self.robot_interface.path_planner.create_path_plan(manipulator_names, waypoints_list)
        except :
            rospy.logerr(str("AffordanceTemplate::plan_path_to_waypoints() -- problem creating path plans"))  
            return False    
    
        for end_effector in end_effectors:
            ret[end_effector] = planner_result[self.robot_interface.get_manipulator(end_effector)]

        # ret = self.robot_interface.path_planner.create_path_plans(manipulator_names, frame_ids, waypoints_list)
        rospy.loginfo(str("AffordanceTemplate::plan_path_to_waypoints() -- finished"))      

        for idx in range(len(ret.keys())) :
            ee_id = ee_ids[idx]
            ee = end_effectors[idx]
            self.waypoint_plan_valid[self.current_trajectory][ee_id] = ret[ee]

            # check for validity
            if not ret[ee] : 
                rospy.logwarn(str("AffordanceTemplate::plan_path_to_waypoints() -- couldnt find valid plan for " + manipulator_names[idx] + " to id: " + next_path_strs[idx]))
                self.path_plan_ids[self.current_trajectory][ee_id] = -1
            else :
                self.path_plan_ids[self.current_trajectory][ee_id] = next_path_idxs[idx]

        return (not False in ret)

    def get_fixed_joint_offset(self, end_effector) :
        T = Frame()
        manipulator_group = self.robot_interface.path_planner.get_srdf_model().group_end_effectors[end_effector].parent_group
        control_frame = self.robot_interface.path_planner.get_srdf_model().group_end_effectors[end_effector].parent_link
        urdf = self.robot_interface.path_planner.get_urdf_model()
        link = control_frame
        while True :
            joint = get_link_joint(link, urdf)
            if urdf.joint_map[joint].type == "fixed" :
                p = joint_origin_to_pose(urdf.joint_map[joint])
                T_new = getFrameFromPose(p)
                T = T*T_new
                link = get_parent_link(link,urdf)
            else :
                break
        return T

    def move_to_waypoints(self, end_effectors) :

        rospy.logwarn(str("AffordanceTemplate::move_to_waypoints()"))

        plan_names = []
        manipulator_names = []
        ee_names = []
        ee_ids = []
        ret = {}
        for end_effector in end_effectors :

            rospy.logwarn(str("AffordanceTemplate::move_to_waypoint() -- " + str(end_effector)))
            ee_id = self.robot_interface.manipulator_id_map[end_effector]
            ee_name = self.robot_interface.get_end_effector_name(ee_id)
            manipulator_name = self.robot_interface.get_manipulator(end_effector)
            ret[manipulator_name] = False
            
            if self.waypoint_plan_valid[self.current_trajectory][ee_id] :
                
                manipulator_names.append(manipulator_name)
                ee_names.append(ee_name)
                ee_ids.append(ee_id)
                
                plan_names.append(manipulator_name)
                plan_names.append(ee_name)


        # first execute for the manipulator (arm)
        ret = self.robot_interface.path_planner.execute(plan_names,from_stored=True, wait=False)

        for ee_name in ee_names :
            del ret[ee_name]

        idx = 0
        for idx in range(len(ee_ids)) :
            manipulator_name = manipulator_names[idx]
            ee_id = ee_ids[idx]
            if not ret[manipulator_name] :
                rospy.logerr(str("AffordanceTemplate::move_to_waypoint() -- failed execution for group: " + manipulator_name + ". re-synching..."))
                self.waypoint_execution_valid[self.current_trajectory][ee_id] = False

            # # next execute for the end-effector (hand)
            # r = self.robot_interface.path_planner.execute(ee_name,from_stored=True, wait=False)
            # if not r :
            #     rospy.logerr(str("AffordanceTemplate::move_to_waypoint() -- failed execution for group: " + ee_name + ". re-synching..."))
            #     return False

            # if succeeded, update the current index, and set the valid plan to False so we dont jump back later
            else :
                rospy.loginfo(str("AffordanceTemplate::move_to_waypoint() -- setting current waypoint idx for " + manipulator_name + " to " + str(self.waypoint_index[self.current_trajectory][ee_id])))
                self.waypoint_index[self.current_trajectory][ee_id] = self.path_plan_ids[self.current_trajectory][ee_id]
                self.path_plan_ids[self.current_trajectory][ee_id] = -1
                self.waypoint_plan_valid[self.current_trajectory][ee_id] = False
                self.waypoint_execution_valid[self.current_trajectory][ee_id] = True

            return ret
            
        else :
            rospy.logerr(str("AffordanceTemplate::move_to_waypoint() -- waypoint plan not valid..."))
            return ret

    def plan_to_waypoints_valid(self, end_effectors, trajectory) :
        
        ret = True

        for end_effector in end_effectors :
            if not end_effector in self.robot_interface.manipulator_id_map.keys() :
                rospy.logerr("AffordanceTemplate::plan_to_waypoint_valid() -- invalid end-effector in id map")
                return False
            
            ee_id = self.robot_interface.manipulator_id_map[end_effector]
            
            if not trajectory in self.waypoint_plan_valid.keys() :
                rospy.logerr("AffordanceTemplate::plan_to_waypoint_valid() -- invalid trajectory in valid map")
                return False

            if not ee_id in self.waypoint_plan_valid[trajectory].keys() :
                rospy.logerr("AffordanceTemplate::plan_to_waypoint_valid() -- invalid end-effector in valid map")
                return False

            ret = ret and self.waypoint_plan_valid[trajectory][ee_id]

        return ret

    def create_origin_from_pose(self, ps) :
        origin = {}
        origin_rpy = (kdl.Rotation.Quaternion(ps.orientation.x,ps.orientation.y,ps.orientation.z,ps.orientation.w)).GetRPY()
        origin['xyz'] = [0]*3
        origin['rpy'] = [0]*3
        origin['xyz'][0] = ps.position.x
        origin['xyz'][1] = ps.position.y
        origin['xyz'][2] = ps.position.z
        origin['rpy'][0] = origin_rpy[0]
        origin['rpy'][1] = origin_rpy[1]
        origin['rpy'][2] = origin_rpy[2]
        return origin

    def create_6dof_controls(self, scale) :
        controls = {}
        controls['xyz'] = [0]*3
        controls['rpy'] = [0]*3
        controls['xyz'][0] = 1
        controls['xyz'][1] = 1
        controls['xyz'][2] = 1
        controls['rpy'][0] = 1
        controls['rpy'][1] = 1
        controls['rpy'][2] = 1
        controls['scale'] = scale
        return controls


    def terminate(self) :
        rospy.loginfo("(EX)TERMINATE!!!!!!!!!")
        self._stop.set()
        self.robot_interface.tear_down()
        rospy.sleep(2)
        self.delete_callback(None)
        self.running = False
        rospy.sleep(.5)


    def run(self) :
        while self.running :
            self.mutex.acquire()
            try :
                try :
                    for obj in self.frame_store_map.keys() :
                       self.tf_broadcaster.sendTransform((self.frame_store_map[obj].pose.position.x,self.frame_store_map[obj].pose.position.y,self.frame_store_map[obj].pose.position.z),
                                                  (self.frame_store_map[obj].pose.orientation.x,self.frame_store_map[obj].pose.orientation.y,self.frame_store_map[obj].pose.orientation.z,self.frame_store_map[obj].pose.orientation.w),
                                                  rospy.Time.now(), self.frame_store_map[obj].frame_id, self.frame_store_map[obj].root_frame_id)
                except :
                    rospy.logdebug("AffordanceTemplate::run() -- could not update thread")
            finally :
                self.mutex.release()

            rospy.sleep(0.1)
        rospy.logdebug(str("AffordanceTemplate::run() -- Killing frame update thread for AT: " + self.name))
