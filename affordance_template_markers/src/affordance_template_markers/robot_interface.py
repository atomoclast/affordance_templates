
import yaml

import rospy
import tf
import PyKDL as kdl

import sensor_msgs.msg

from geometry_msgs.msg import Pose, PoseArray
from affordance_template_msgs.msg import *

from nasa_robot_teleop.path_planner import *
from nasa_robot_teleop.util.kinematics_util import *
from nasa_robot_teleop.pose_update_thread import *
from nasa_robot_teleop.end_effector_helper import *

class RobotInterface(object) :
    
    def __init__(self, joint_states_topic="/joint_states", yaml_file=None, robot_config=None) :
    
        self.reset()
    
        self.tf_listener = tf.TransformListener()
        rospy.Subscriber(joint_states_topic, sensor_msgs.msg.JointState, self.joint_state_callback)
        self.path_planner = None

        if yaml_file and robot_config :
            rospy.logerr("RobotInterface() -- can't load from both yaml and robot_config!")
        elif yaml_file :
            rospy.loginfo(("RobotInterface() -- loading with input yaml: " + yaml_file))
            self.load_from_file(yaml_file)
        elif robot_config :
            rospy.loginfo(("RobotInterface() -- loading from input robot config: " + robot_config.name))
            self.load_from_msg(robot_config)
        else:
            rospy.logwarn("RobotInterface() -- no robot configuration given...")

    def reset(self) :
        self.robot_config = RobotConfig()
        self.ee_groups = []
        self.end_effector_names = []
        self.end_effector_name_map = {}
        self.manipulator_pose_map = {}
        self.manipulator_id_map = {}
        self.tool_offset_map = {}
        self.end_effector_pose_map = {}
        self.end_effector_id_map = {}
        self.end_effector_link_data = {}
        self.end_effector_markers = {}
        self.stored_poses = {}
        self.gripper_service = None
        self.configured = False

    def load_from_msg(self, robot_config) :

        self.reset()
        self.robot_config = robot_config

        rospy.set_param("/affordance_templates/robot_yaml", "")

        for ee_config in self.robot_config.end_effectors:
            self.manipulator_pose_map[ee_config.name] = ee_config.pose_offset
            self.tool_offset_map[ee_config.name] = ee_config.tool_offset
            self.end_effector_names.append(ee_config.name)
            self.end_effector_name_map[ee_config.id] = ee_config.name
            self.manipulator_id_map[ee_config.name] = ee_config.id

        for ee_pose in self.robot_config.end_effector_pose_data :
            if not ee_pose.group in self.end_effector_pose_map:
                self.end_effector_pose_map[ee_pose.group] = {}
                self.end_effector_id_map[ee_pose.group] = {}
            self.end_effector_pose_map[ee_pose.group][ee_pose.name] = int(ee_pose.id)
            self.end_effector_id_map[ee_pose.group][int(ee_pose.id)] = ee_pose.name

        self.configured = True

        return True

    def load_from_file(self, filename) :

        self.reset()

        try:
        
            f = open(filename)
            self.yaml_config = yaml.load(f.read())
            f.close()
              
            yaml_stuff = filename.split("/")
            yaml_base = yaml_stuff[len(yaml_stuff)-1]
            rospy.set_param("/affordance_templates/robot_yaml", yaml_base)

            self.robot_config.filename  = filename
            self.robot_config.name = self.yaml_config['robot_name']
            self.robot_config.config_package = str(self.yaml_config['config_package'])
            self.robot_config.config_file = str(self.yaml_config['config_file'])
            self.robot_config.planner_type = str(self.yaml_config['planner_type'])
            self.robot_config.frame_id = self.yaml_config['frame_id']
            
            q = (kdl.Rotation.RPY(self.yaml_config['root_offset'][3],self.yaml_config['root_offset'][4],self.yaml_config['root_offset'][5])).GetQuaternion()
            self.robot_config.root_offset = Pose()
            self.robot_config.root_offset.position.x = self.yaml_config['root_offset'][0]
            self.robot_config.root_offset.position.y = self.yaml_config['root_offset'][1]
            self.robot_config.root_offset.position.z = self.yaml_config['root_offset'][2]
            self.robot_config.root_offset.orientation.x = q[0]
            self.robot_config.root_offset.orientation.y = q[1]
            self.robot_config.root_offset.orientation.z = q[2]
            self.robot_config.root_offset.orientation.w = q[3]

            for ee in self.yaml_config['end_effector_group_map']:

                ee_config = EndEffectorConfig()
                ee_config.name = ee['name']
                ee_config.id = ee['id']
                ee_config.pose_offset = Pose()

                try :
                    q = (kdl.Rotation.RPY(ee['pose_offset'][3],ee['pose_offset'][4],ee['pose_offset'][5])).GetQuaternion()
                    ee_config.pose_offset.position.x = float(ee['pose_offset'][0])
                    ee_config.pose_offset.position.y = float(ee['pose_offset'][1])
                    ee_config.pose_offset.position.z = float(ee['pose_offset'][2])
                    ee_config.pose_offset.orientation.x = q[0]
                    ee_config.pose_offset.orientation.y = q[1]
                    ee_config.pose_offset.orientation.z = q[2]
                    ee_config.pose_offset.orientation.w = q[3]
                except :
                    ee_config.pose_offset.orientation.w = 1.0
                self.manipulator_pose_map[ee['name']] = ee_config.pose_offset
                
                ee_config.tool_offset = Pose()
                try :
                    q = (kdl.Rotation.RPY(ee['tool_offset'][3],ee['tool_offset'][4],ee['tool_offset'][5])).GetQuaternion()
                    ee_config.tool_offset.position.x = float(ee['tool_offset'][0])
                    ee_config.tool_offset.position.y = float(ee['tool_offset'][1])
                    ee_config.tool_offset.position.z = float(ee['tool_offset'][2])
                    ee_config.tool_offset.orientation.x = q[0]
                    ee_config.tool_offset.orientation.y = q[1]
                    ee_config.tool_offset.orientation.z = q[2]
                    ee_config.tool_offset.orientation.w = q[3]
                except :
                    ee_config.tool_offset.orientation.w = 1.0
                self.tool_offset_map[ee['name']] = ee_config.tool_offset

                self.end_effector_names.append(ee['name'])
                self.end_effector_name_map[ee['id']] = ee['name']
                self.manipulator_id_map[ee['name']] = ee['id']
                
                self.robot_config.end_effectors.append(ee_config)

                # if there is a tool offset
                # t = geometry_msgs.msg.Pose()
                # try :
                #     q = (kdl.Rotation.RPY(ee['tool_offset'][3],ee['tool_offset'][4],ee['tool_offset'][5])).GetQuaternion()
                #     t.position.x = float(ee['tool_offset'][0])
                #     t.position.y = float(ee['tool_offset'][1])
                #     t.position.z = float(ee['tool_offset'][2])
                #     t.orientation.x = q[0]
                #     t.orientation.y = q[1]
                #     t.orientation.z = q[2]
                #     t.orientation.w = q[3]
                # except :
                #     t.orientation.w = 1.0
                # self.tool_offset_map[ee['name']] = t

            for ee in self.yaml_config['end_effector_pose_map']:

                ee_pose_config = EndEffectorPoseData()

                ee_pose_config.group = ee['group']
                ee_pose_config.name = ee['name']
                ee_pose_config.id = ee['id']
                
                self.robot_config.end_effector_pose_data.append(ee_pose_config)

                if not ee['group'] in self.end_effector_pose_map:
                    self.end_effector_pose_map[ee['group']] = {}
                    self.end_effector_id_map[ee['group']] = {}
                self.end_effector_pose_map[ee['group']][ee['name']] = int(ee['id'])
                self.end_effector_id_map[ee['group']][int(ee['id'])] = ee['name']
                
                # print "ee[", ee['group'], "] adding group [", ee['name'], "] with id [", ee['id'], "]"
            

            try :
                gs = self.yaml_config['gripper_service']
                self.robot_config.gripper_service = gs
                rospy.loginfo(str("RobotInterface() -- found gripper service : " + self.robot_config.gripper_service))
            except :
                self.robot_config.gripper_service = ""
                
            rospy.logdebug(str("RobotInterface::load_from_file() -- loaded: " + filename))

        except :
            rospy.logerr("RobotInterface::load_from_file() -- error opening config file")
            return False

        # print self.robot_config
        self.config_file = self.get_package_path(self.robot_config.config_package) + "/" + str(self.robot_config.config_file)
        print self.config_file
        self.configured = True

        return True

    def configure_path_planner(self) :
        # planner instantiatation
        if self.robot_config.planner_type == "moveit" :
            from nasa_robot_teleop.planners.moveit_path_planner import MoveItPathPlanner
            self.path_planner = MoveItPathPlanner(self.robot_config.name, self.config_file)
        elif self.robot_config.planner_type == "atlas" :
            from nasa_robot_teleop.planners.atlas_path_planner import AtlasPathPlanner
            self.path_planner = AtlasPathPlanner(self.robot_config.name, self.config_file)
        else :
            rospy.logerr("InteractiveControl() unrecognized planner type!!")
            return False
        return True

    def configure(self) :
        rospy.loginfo(str("\n--------------------------\nRobotInterface::configure()"))
        rospy.loginfo(str("RobotInterface::configure() -- configuring for robot name: " + self.robot_config.name))
        rospy.loginfo(str("RobotInterface::configure() -- configuring for package: " + self.config_file))
        rospy.loginfo(str("RobotInterface::configure() -- planner type: " + self.robot_config.planner_type))
        

        self.configure_path_planner()     


        self.root_frame = self.path_planner.get_robot_planning_frame()
        self.ee_groups = self.path_planner.get_end_effector_names()
        for g in self.end_effector_names :
    
            if not g in self.ee_groups:
                rospy.logerr("RobotInterface::configure() -- group " + str(g) + " not in end effector groups!")
                return False
            else :
                if self.path_planner.add_planning_group(g, "endeffector") :
                    self.path_planner.set_goal_joint_tolerance(g, 0.05)
                    rospy.loginfo(str("RobotInterface::configure() -- control frame: " + self.path_planner.get_control_frame(g)))
                    self.end_effector_link_data[g] = EndEffectorHelper(self.robot_config.name, g, self.path_planner.get_srdf_model().get_end_effector_link(g), self.tf_listener)
                    self.end_effector_link_data[g].populate_data(self.path_planner.get_group_links(g), self.path_planner.get_urdf_model(), self.path_planner.get_srdf_model())
                    rospy.sleep(2)
                    self.end_effector_markers[g] = self.end_effector_link_data[g].get_current_position_marker_array(scale=1.0,color=(1,1,1,0.5))
                    pg = self.path_planner.get_srdf_model().get_end_effector_parent_group(g)
                    if not pg == None :
                        rospy.loginfo(str("RobotInterface::configure() -- trying to add group: " + pg))
                        self.path_planner.add_planning_group(pg, group_type="cartesian")
                        self.path_planner.set_display_mode(pg, "all_points")
                        self.path_planner.set_goal_position_tolerances(pg, [.005]*3)
                        self.path_planner.set_goal_orientation_tolerances(pg, [.02]*3)
                        self.path_planner.set_goal_joint_tolerance(pg, 0.05)

                        self.stored_poses[pg] = {}
                        for state_name in self.path_planner.get_stored_state_list(pg) :
                            rospy.loginfo(str("RobotInterface::configure() adding stored pose \'" + state_name + "\' to group \'" + pg + "\'"))
                            self.stored_poses[pg][state_name] = self.path_planner.get_stored_group_state(pg, state_name)

                    else :
                        rospy.logerror(str("RobotInterface::configure -- no manipulator group found for end-effector: " + g))
                else :
                    rospy.logerr(str("RobotInterface::configure() -- problem adding end-effector group[" + g + "] to planner"))
                    return False
            self.stored_poses[g] = {}
            for state_name in self.path_planner.get_stored_state_list(g) :
                rospy.logdebug(str("RobotInterface::configure() adding stored pose \'" + state_name + "\' to group \'" + g + "\'"))
                self.stored_poses[g][state_name] = self.path_planner.get_stored_group_state(g, state_name)


        if self.robot_config.gripper_service :
            self.path_planner.set_gripper_service(self.robot_config.gripper_service)

        # what do we have?
        rospy.logdebug(str("RobotInterface::configure() -- groups: " + self.path_planner.groups.keys()))
        rospy.logdebug(str("RobotInterface::configure() -- end_effector_names: " + self.end_effector_names))
        rospy.logdebug(str("RobotInterface::configure() -- end_effector_groups: " + self.ee_groups))
        
        self.path_planner.print_basic_info()
        rospy.loginfo(str("RobotInterface::configure() -- end\n--------------------------\n"))
        return True

    def joint_state_callback(self, data) :
        self.joint_data = data

    def get_end_effector_name(self, id) :
        return self.end_effector_name_map[id]

    def get_end_effector_id(self, name) :
        return self.manipulator_id_map[name]

    def get_manipulator(self, ee) :
        return self.path_planner.get_srdf_model().get_end_effector_parent_group(ee)

    def get_package_path(self, pkg):
        """Return the path to the ROS package."""
        import rospkg
        try:
            rp = rospkg.RosPack()
            return rp.get_path(pkg)
        except:
            rospy.logwarn(str("RobotInterface::get_package_path() -- can't find package: " + str(pkg)))
            return ""

    def tear_down(self) :
        for k in self.end_effector_link_data.keys() :
            self.end_effector_link_data[k].stop_offset_update_thread()
        self.path_planner.tear_down()
        self.configured = False
        

