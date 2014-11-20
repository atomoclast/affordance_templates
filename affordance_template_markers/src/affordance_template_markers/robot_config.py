
import yaml

import rospy
import tf
import PyKDL as kdl

import geometry_msgs.msg
import sensor_msgs.msg

from nasa_robot_teleop.moveit_interface import *
from nasa_robot_teleop.kdl_posemath import *
from nasa_robot_teleop.pose_update_thread import *
from nasa_robot_teleop.end_effector_helper import *

class RobotConfig(object) :
    def __init__(self) :
        self.reset()
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.joint_state_callback)

    def reset(self) :
        self.robot_name = ""
        self.config_package =  ""
        self.moveit_ee_groups = []
        self.end_effector_names = []
        self.end_effector_name_map = {}
        self.manipulator_pose_map = {}
        self.manipulator_id_map = {}
        self.tool_offset_map = {}
        self.end_effector_pose_map = {}
        self.end_effector_id_map = {}
        self.end_effector_link_data = {}
        self.end_effector_markers = {}
        self.frame_id = "world"
        self.root_offset = geometry_msgs.msg.Pose()
        self.stored_poses = {}
        self.gripper_service = None

    def load_from_file(self, filename) :

        try:
            f = open(filename)
            self.yaml_config = yaml.load(f.read())
            f.close()
            self.print_yaml()

            self.robot_name = self.yaml_config['robot_name']
            self.config_package = str(self.yaml_config['moveit_config_package'])
            self.frame_id = self.yaml_config['frame_id']

            q = (kdl.Rotation.RPY(self.yaml_config['root_offset'][3],self.yaml_config['root_offset'][4],self.yaml_config['root_offset'][5])).GetQuaternion()
            self.root_offset.position.x = self.yaml_config['root_offset'][0]
            self.root_offset.position.y = self.yaml_config['root_offset'][1]
            self.root_offset.position.z = self.yaml_config['root_offset'][2]
            self.root_offset.orientation.x = q[0]
            self.root_offset.orientation.y = q[1]
            self.root_offset.orientation.z = q[2]
            self.root_offset.orientation.w = q[3]

            for ee in self.yaml_config['end_effector_group_map']:
                self.end_effector_names.append(ee['name'])
                self.end_effector_name_map[ee['id']] = ee['name']
                self.manipulator_id_map[ee['name']] = ee['id']
                p = geometry_msgs.msg.Pose()
                q = (kdl.Rotation.RPY(ee['pose_offset'][3],ee['pose_offset'][4],ee['pose_offset'][5])).GetQuaternion()
                p.position.x = float(ee['pose_offset'][0])
                p.position.y = float(ee['pose_offset'][1])
                p.position.z = float(ee['pose_offset'][2])
                p.orientation.x = q[0]
                p.orientation.y = q[1]
                p.orientation.z = q[2]
                p.orientation.w = q[3]
                self.manipulator_pose_map[ee['name']] = p

                # if there is a tool offset
                t = geometry_msgs.msg.Pose()
                try :
                    q = (kdl.Rotation.RPY(ee['tool_offset'][3],ee['tool_offset'][4],ee['tool_offset'][5])).GetQuaternion()
                    t.position.x = float(ee['tool_offset'][0])
                    t.position.y = float(ee['tool_offset'][1])
                    t.position.z = float(ee['tool_offset'][2])
                    t.orientation.x = q[0]
                    t.orientation.y = q[1]
                    t.orientation.z = q[2]
                    t.orientation.w = q[3]
                except :
                    t.orientation.w = 1.0
                self.tool_offset_map[ee['name']] = t

            try :
                for ee in self.yaml_config['end_effector_pose_map']:
                    if not ee['group'] in self.end_effector_pose_map:
                        self.end_effector_pose_map[ee['group']] = {}
                        self.end_effector_id_map[ee['group']] = {}
                    self.end_effector_pose_map[ee['group']][ee['name']] = int(ee['id'])
                    self.end_effector_id_map[ee['group']][int(ee['id'])] = ee['name']
                    print "ee[", ee['group'], "] adding group [", ee['name'], "] with id [", ee['id'], "]"
            except :
                rospy.logwarn("RobotConfig::load_from_file() -- no end effector pose map found ")
            

            try :
                gs = self.yaml_config['gripper_service']
                self.gripper_service = gs
            except :
                self.gripper_service = ""
                
            print "Robot Config found gripper service : ", self.gripper_service

        except :
            rospy.logerr("RobotConfig::load_from_file() -- error opening config file")
            return False

        return True

    def configure(self) :
        # self.moveit_interface_threads = {}
        self.moveit_interface = MoveItInterface(self.robot_name,self.config_package)
        self.root_frame = self.moveit_interface.get_planning_frame()
        self.moveit_ee_groups = self.moveit_interface.srdf_model.get_end_effector_groups()
        print "RobotConfig::configure -- moveit groups: ", self.moveit_interface.groups
        print "RobotConfig::configure -- end_effector_names: ",self.end_effector_names
        for g in self.end_effector_names :
            if not g in self.moveit_ee_groups:
                rospy.logerr("RobotConfig::configure() -- group ", g, " not in moveit end effector groups!")
                return False
            else :

                # self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.get_control_frame(g), self.tf_listener)
                print "control frame: ", self.moveit_interface.srdf_model.group_end_effectors[g].parent_link
                self.end_effector_link_data[g] = EndEffectorHelper(self.robot_name, g, self.moveit_interface.srdf_model.group_end_effectors[g].parent_link, self.tf_listener)
                self.end_effector_link_data[g].populate_data(self.moveit_interface.get_group_links(g), self.moveit_interface.get_urdf_model(), self.moveit_interface.get_srdf_model())
                rospy.sleep(2)
                self.end_effector_markers[g] = self.end_effector_link_data[g].get_current_position_marker_array(scale=1.0,color=(1,1,1,0.5))
                pg = self.moveit_interface.srdf_model.get_end_effector_parent_group(g)
                if not pg == None :
                    print "trying to add MoveIt! group: ", pg
                    self.moveit_interface.add_group(pg, group_type="manipulator")
                    self.moveit_interface.set_display_mode(pg, "all_points")
                    # self.moveit_interface_threads[pg] = MoveItInterfaceThread(self.robot_name,self.config_package, g)

                    self.stored_poses[pg] = {}
                    for state_name in self.moveit_interface.get_stored_state_list(pg) :
                        rospy.loginfo(str("RobotConfig::configure() adding stored pose \'" + state_name + "\' to group \'" + pg + "\'"))
                        self.stored_poses[pg][state_name] = self.moveit_interface.get_stored_group_state(pg, state_name)


                else :
                    print "no manipulator group found for end-effector: ", g

            self.stored_poses[g] = {}
            for state_name in self.moveit_interface.get_stored_state_list(g) :
                rospy.loginfo(str("RobotConfig::configure() adding stored pose \'" + state_name + "\' to group \'" + g + "\'"))
                self.stored_poses[g][state_name] = self.moveit_interface.get_stored_group_state(g, state_name)


        if self.gripper_service :
            self.moveit_interface.set_gripper_service(self.gripper_service)

        # what do we have?
        self.moveit_interface.print_basic_info()
        return True

    def print_yaml(self) :
        if not self.yaml_config == None:
            print "============================="
            print "Robot Config Info: "
            print "============================="
            print " robot name: ", self.yaml_config['robot_name']
            print " root offset: ", self.yaml_config['root_offset']
            print " moveit_config: ", self.yaml_config['moveit_config_package']
            for ee in self.yaml_config['end_effector_group_map']:
                print "\t", "map: ", ee['name'], " --> ", ee['id']
                print "\t", "pose_offset: ", ee['pose_offset']
                try :
                    ee['tool_offset']
                    print "\t", "tool_offset: ", ee['tool_offset']
                except :
                    pass


            try :
                print " gripper service: ", self.yaml_config['gripper_service']
            except :
                pass
            print "============================="

    def joint_state_callback(self, data) :
        self.joint_data = data

    def get_end_effector_name(self, id) :
        return self.end_effector_name_map[id]

    def get_manipulator(self, ee) :
        return self.moveit_interface.srdf_model.get_end_effector_parent_group(ee)

    def tear_down(self) :
        for k in self.end_effector_link_data.keys() :
            self.end_effector_link_data[k].stop_offset_update_thread()
        self.moveit_interface.tear_down()

