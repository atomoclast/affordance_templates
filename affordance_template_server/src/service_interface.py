import rospy

from affordance_template_msgs.msg import *  
from affordance_template_msgs.srv import *  

class ServiceInterface(object):

    def __init__(self, server):
        
        rospy.loginfo("ServiceInterface() starting")
        # the affordance template server
        self.server = server

        # services
        self.robot_info_service =      rospy.Service('/affordance_template_server/get_robots', GetRobotConfigInfo, self.handle_robot_request)
        self.template_info_service =   rospy.Service('/affordance_template_server/get_templates', GetAffordanceTemplateConfigInfo, self.handle_template_request)
        self.object_info_service =     rospy.Service('/affordance_template_server/get_recognition_objects', GetRecognitionObjectConfigInfo, self.handle_object_request)
        self.load_robot_service =      rospy.Service('/affordance_template_server/load_robot', LoadRobotConfig, self.handle_load_robot)
        self.add_template_service =    rospy.Service('/affordance_template_server/add_template', AddAffordanceTemplate, self.handle_add_template)
        self.delete_template_service = rospy.Service('/affordance_template_server/delete_template', DeleteAffordanceTemplate, self.handle_template_kill)
        self.add_object_service =      rospy.Service('/affordance_template_server/add_recognition_object', AddRecognitionObject, self.handle_add_object)
        self.delete_object_service =   rospy.Service('/affordance_template_server/delete_recognition_object', DeleteRecognitionObject, self.handle_object_kill)
        self.get_running_service =     rospy.Service('/affordance_template_server/get_running', GetRunningAffordanceTemplates, self.handle_running)
        self.command_service =         rospy.Service('/affordance_template_server/command', AffordanceTemplateCommand, self.handle_command)


    def handle_robot_request(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_robot_request() -- requested robot info " + request.name))
        response = GetRobotConfigInfoResponse()
        if request.name and request.name in self.server.robot_map.keys() :
            response.robots.append(self.server.robot_map[request.name].robot_config)
        else :
            for r in self.server.robot_map.keys() :
                response.robots.append(self.server.robot_map[r].robot_config)
        return response

    def handle_template_request(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_template_request() -- requested template info " + request.name))
        response = GetAffordanceTemplateConfigInfoResponse()
        if request.name and request.name in self.server.at_data.class_map.keys() :
            class_type = request.name
            at_config = AffordanceTemplateConfig()
            at_config.type = class_type
            at_config.image_path = self.server.at_data.image_map[class_type]
            for t in self.server.at_data.traj_map[class_type] :
                wp_traj = WaypointTrajectory()
                wp_traj.name = t
                for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
                    wp = WaypointInfo()
                    wp.id = int(p)
                    wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]
                    wp_traj.waypoint_info.append(wp)
                at_config.trajectory_info.append(wp_traj)
            response.templates.append(at_config)
        else :
            for class_type in self.server.at_data.class_map.keys():
                at_config = AffordanceTemplateConfig()
                at_config.type = class_type
                at_config.image_path = self.server.at_data.image_map[class_type]
                for t in self.server.at_data.traj_map[class_type] :
                    wp_traj = WaypointTrajectory()
                    wp_traj.name = t
                    for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
                        wp = WaypointInfo()
                        wp.id = int(p)
                        wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]
                        wp_traj.waypoint_info.append(wp)
                    at_config.trajectory_info.append(wp_traj)
                response.templates.append(at_config)
        return response

    def handle_template_request(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_object_request() -- requested object info " + request.name))
        rospy.logwarn(str("ServiceInterface::handle_object_request() -- not implemeneted yet"))
        response = GetRecognitionObjectConfigInfoResponse()
        return response

    def handle_load_robot(self, request):
        rospy.loginfo(str("ServiceInterface::handle_load_robot() -- load request for robot " + request.robot_config.name))
        response = LoadRobotConfigResponse()
        response.status = False
        try:
            if self.server.robot_interface.configured :
                self.server.robot_interface.tear_down() 

            if request.filename :
                self.server.robot_interface.load_from_msg(request.filename)
            else :
                self.server.robot_interface.load_from_msg(request.robot_config)

            response.status = self.server.robot_interface.configure()
        except:
            rospy.logerror("ServiceInterface::handle_load_robot()  -- Error trying to load robot from message")
        return response

    def handle_add_template(self, request):
        rospy.loginfo(str("ServiceInterface::handle_add_template() -- adding template " + request.class_type))
        response = AddAffordanceTemplateResponse()
        response.status = False
        try:
            pid = self.server.get_next_template_id(request.class_type)
            response.id = pid
            response.status = self.server.add_template(request.class_type, response.id)
        except:
            rospy.logerr("ServiceInterface::handle_add_template() -- error adding template to server")
        return response

    def handle_add_object(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_add_object() -- add object of type " + request.object_type))
        rospy.logwarn(str("ServiceInterface::handle_add_object() -- not implemeneted yet"))
        response = AddRecognitionObjectResponse()
        return response

    def handle_running(self, request):
        rospy.loginfo("ServiceInterface::handle_running()")
        response = GetRunningAffordanceTemplatesResponse()
        try:
            for t in self.server.at_data.class_map.keys():
                for i in self.server.at_data.class_map[t].keys():
                    response.templates.append(t + ":" + str(i))
            response.templates.sort()
        except:
            rospy.logerr("ServiceInterface::handle_running() -- error getting running templates")
        return response

    def handle_template_kill(self, request):
        rospy.loginfo(str("ServiceInterface::handle_template_kill() -- killing template " + request.class_type + "[" + str(request.id) + "]"))
        response = DeleteAffordanceTemplateResponse()
        response.status = False
        try:
            response.status = self.server.remove_template(request.class_type, request.id)
        except:
            rospy.logerr("ServiceInterface::handle_template_kill() -- error deleting requested templates")
        return response

    def handle_object_kill(self, request) :
        rospy.loginfo(str("ServiceInterface::handle_object_kill() -- delete object of type " + request.object_type))
        rospy.logwarn(str("ServiceInterface::handle_object_kill() -- not implemeneted yet"))
        response = DeleteRecognitionObjectResponse()
        return response


    def handle_command(self, request):
        rospy.loginfo(str("ServiceInterface::handle_command() -- new command request for template " + request.type + "[" + str(request.id) + "]: " + str(request.command)))        
        response = AffordanceTemplateCommandResponse()
        response.status = False

        try:
            
            idx = {}
            at = self.server.at_data.class_map[request.type][int(request.id)]
            # plan first
            for ee in request.end_effectors:

                if not at.trajectory_has_ee(at.current_trajectory, ee): 
                    rospy.logwarn(str("ServiceInterface::handle_command() -- " + ee + " not in trajectory, can't plan"))
                    continue

                if request.command == request.GO_TO_START :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=True)
                elif request.command == request.GO_TO_END :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), steps=999, direct=True)
                elif request.command == request.PLAY_BACKWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), backwards=True, steps=-999, direct=False)
                elif request.command == request.PLAY_FORWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), steps=999, direct=False)
                elif request.command == request.STEP_BACKWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), backwards=True, steps=request.steps)
                elif request.command == request.STEP_FORWARD :
                    idx[ee] = at.plan_path_to_waypoint(str(ee), steps=request.steps)
                elif request.command == request.STOP :
                    at.stop(str(ee))

                rospy.loginfo(str("ServiceInterface::handle_command() -- done planning path for " + ee))

            # execute after
            for ee in request.end_effectors :

                if not at.trajectory_has_ee(at.current_trajectory, ee): 
                    rospy.logwarn(str("ServiceInterface::handle_command() -- " + ee + " not in trajectory, can't execute"))
                    continue
                if request.execute_on_plan:        
                    rospy.loginfo(str("ServiceInterface::handle_command() -- executing path for " + ee + ", precomputed: " + str(request.execute_precomputed_plan)))
                    at.move_to_waypoint(str(ee), idx[ee])
                    wp = WaypointInfo()
                    wp.id = int(at.robot_interface.manipulator_id_map[str(ee)])
                    wp.waypoint_index = idx[ee]
                    wp.num_waypoints = at.waypoint_max[at.current_trajectory][wp.id]+1
                    response.waypoint_info.append(wp)
                else :
                    wp = WaypointInfo()
                    wp.id = int(at.robot_interface.manipulator_id_map[str(ee)])
                    wp.waypoint_index = int(at.waypoint_index[at.current_trajectory][wp.id])
                    wp.num_waypoints = at.waypoint_max[at.current_trajectory][wp.id]+1
                    response.waypoint_info.append(wp)                  

                rospy.loginfo(str("ServiceInterface::handle_command() -- done execution path for " + ee))
            response.status = True
        except:
            rospy.logerr(str("ServiceInterface::handle_command() -- error performing command!!"))
        return response


