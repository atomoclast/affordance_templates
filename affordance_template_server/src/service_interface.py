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
        # self.object_info_service =     rospy.Service('/affordance_template_server/get_recognition_objects', GetRecognitionObjectConfigInfo, self.handle_object_request)
        self.load_robot_service =      rospy.Service('/affordance_template_server/load_robot', LoadRobotConfig, self.handle_load_robot)
        self.add_template_service =    rospy.Service('/affordance_template_server/add_template', AddAffordanceTemplate, self.handle_add_template)
        self.delete_template_service = rospy.Service('/affordance_template_server/delete_template', DeleteAffordanceTemplate, self.handle_template_kill)
        # self.add_object_service =      rospy.Service('/affordance_template_server/add_recognition_object', AddRecognitionObject, self.handle_add_object)
        # self.delete_object_service =   rospy.Service('/affordance_template_server/delete_recognition_object', DeleteRecognitionObject, self.handle_object_kill)
        self.get_running_service =     rospy.Service('/affordance_template_server/get_running', GetRunningAffordanceTemplates, self.handle_running)
        self.plan_command_service =    rospy.Service('/affordance_template_server/plan_command', AffordanceTemplatePlanCommand, self.handle_plan_command)
        self.execute_command_service = rospy.Service('/affordance_template_server/execute_command', AffordanceTemplateExecuteCommand, self.handle_execute_command)
        self.save_service =            rospy.Service('/affordance_template_server/save_template', SaveAffordanceTemplate, self.handle_save_template)
        self.add_trajectory =          rospy.Service('/affordance_template_server/add_trajectory', AddAffordanceTemplateTrajectory, self.handle_add_trajectory)
        self.scale_object =            rospy.Service('/affordance_template_server/scale_object', ScaleDisplayObject, self.handle_object_scale)
        self.template_status_service = rospy.Service('/affordance_template_server/get_template_status', GetAffordanceTemplateStatus, self.handle_template_status_request)

        # subscribers
        self.scale_object_stream =     rospy.Subscriber('/affordance_template_server/scale_object_streamer', ScaleDisplayObjectInfo, self.handle_object_scale_stream)

        self.stored_idx = {}

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
            at_config.filename = self.server.at_data.file_map[class_type]
            for t in self.server.at_data.traj_map[class_type] :
                wp_traj = WaypointTrajectory()
                wp_traj.name = t
                for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
                    wp = WaypointInfo()
                    wp.id = int(p)
                    wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]
                    wp_traj.waypoint_info.append(wp)
                at_config.trajectory_info.append(wp_traj)
            for obj in self.server.at_data.object_map[class_type] :
                at_config.display_objects.append(obj)
            response.templates.append(at_config)
        else :
            for class_type in self.server.at_data.class_map.keys():
                at_config = AffordanceTemplateConfig()
                at_config.type = class_type
                at_config.image_path = self.server.at_data.image_map[class_type]
                at_config.filename = self.server.at_data.file_map[class_type]
                for t in self.server.at_data.traj_map[class_type] :
                    wp_traj = WaypointTrajectory()
                    wp_traj.name = t
                    for p in self.server.at_data.waypoint_map[(class_type,t)].keys() :
                        wp = WaypointInfo()
                        wp.id = int(p)
                        wp.num_waypoints = self.server.at_data.waypoint_map[(class_type,t)][p]
                        wp_traj.waypoint_info.append(wp)
                    at_config.trajectory_info.append(wp_traj)
                for obj in self.server.at_data.object_map[class_type] :
                    at_config.display_objects.append(obj)
                response.templates.append(at_config)
        return response

    # def handle_object_request(self, request) :
    #     rospy.loginfo(str("ServiceInterface::handle_object_request() -- requested object info " + request.name))
    #     response = GetRecognitionObjectConfigInfoResponse()
    #     if request.name and request.name in self.server.ro_data.object_map.keys() :
    #         object_type = request.name
    #         ro_config = RecognitionObjectConfig()
    #         ro_config.type = object_type
    #         ro_config.image_path = self.server.ro_data.image_map[object_type]
    #         ro_config.package = self.server.ro_data.package_map[object_type]
    #         ro_config.launch_file = self.server.ro_data.launch_map[object_type]
    #         ro_config.marker_topic = self.server.ro_data.marker_topic_map[object_type]
    #         response.recognition_objects.append(ro_config)
    #     else :
    #         for object_type in self.server.ro_data.object_map.keys():
    #             ro_config = RecognitionObjectConfig()
    #             ro_config.type = object_type
    #             ro_config.image_path = self.server.ro_data.image_map[object_type]
    #             ro_config.package = self.server.ro_data.package_map[object_type]
    #             ro_config.launch_file = self.server.ro_data.launch_map[object_type]
    #             ro_config.marker_topic = self.server.ro_data.marker_topic_map[object_type]
    #             response.recognition_objects.append(ro_config)
    #     return response

    def handle_load_robot(self, request):
        rospy.loginfo(str("ServiceInterface::handle_load_robot() -- load request for robot " + request.robot_config.name))
        response = LoadRobotConfigResponse()
        response.status = False
        try:
            if self.server.robot_interface.configured :
                self.server.robot_interface.tear_down() 
            if request.filename :
                self.server.robot_interface.load_from_file(request.filename)
            else :
                self.server.robot_interface.load_from_msg(request.robot_config)
            response.status = self.server.robot_interface.configure()
        except:
            rospy.logerr("ServiceInterface::handle_load_robot()  -- Error trying to load robot from message")
        return response

    def handle_add_template(self, request):
        rospy.loginfo(str("ServiceInterface::handle_add_template() -- adding template " + request.class_type))
        response = AddAffordanceTemplateResponse()
        response.status = False
        try:
            pid = self.server.get_next_template_id(request.class_type)
            response.id = pid
            response.status = self.server.add_template(request.class_type, pid)
        except:
            rospy.logerr("ServiceInterface::handle_add_template() -- error adding template to server")
        return response

    # def handle_add_object(self, request) :
    #     rospy.loginfo(str("ServiceInterface::handle_add_object() -- add object of type " + request.object_type))
    #     rospy.logwarn(str("ServiceInterface::handle_add_object() -- not implemeneted yet"))
    #     response = AddRecognitionObjectResponse()
    #     return response

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

    # def handle_object_kill(self, request) :
    #     rospy.loginfo(str("ServiceInterface::handle_object_kill() -- delete object of type " + request.object_type))
    #     rospy.logwarn(str("ServiceInterface::handle_object_kill() -- not implemeneted yet"))
    #     response = DeleteRecognitionObjectResponse()
    #     return response

    def handle_plan_command(self, request):
        
        rospy.loginfo(str("ServiceInterface::handle_plan_command() -- new plan request for  " + request.type + ":" + str(request.id) + ", trajectory " + str(request.trajectory_name)))        
        response = AffordanceTemplatePlanCommandResponse()
        response.status = False
        r = {}

        try:          
            
            # get the AT instance
            at = self.server.at_data.class_map[request.type][int(request.id)]
            
            # check if a specific trajectory was given
            if request.trajectory_name == "":
                request.trajectory_name = at.current_trajectory
                        
            # go through all the EE waypoints in the request
            id = 0
            for ee in request.end_effectors:
                                
                steps = int(request.steps[id])
                # make sure the EE is in the trajectory
                if not at.trajectory_has_ee(request.trajectory_name, ee): 
                    rospy.logwarn(str("ServiceInterface::handle_plan_command() -- " + ee + " not in trajectory, can't plan"))
                    continue
                                
                # compute path plan
                r[ee] = at.plan_path_to_waypoint(str(ee), steps=steps, direct=request.direct, backwards=request.backwards)
                rospy.loginfo(str("ServiceInterface::handle_plan_command() -- done planning path for " + ee))
                
                id += 1

            # status will return True only when all ee's in plan are valid
            response.status = (not False in r.values()) 
            response.affordance_template_status = self.get_template_status(request.type, request.id, request.trajectory_name)
    
        except:
            rospy.logerr(str("ServiceInterface::handle_plan_command() -- error performing command!!"))
        
        return response


    def handle_execute_command(self, request):

        rospy.loginfo(str("ServiceInterface::handle_execute_command() -- new execute request for  " + request.type + ":" + str(request.id) + ", trajectory " + str(request.trajectory_name)))        
       
        response = AffordanceTemplateExecuteCommandResponse()
        response.status = False
        r = {}
        
        try:          
            
            # get the AT instance
            at = self.server.at_data.class_map[request.type][int(request.id)]
            
            # check if a specific trajectory was given
            if request.trajectory_name == "":
                request.trajectory_name = at.current_trajectory

            # go through all the EE waypoints in the request
            for ee in request.end_effectors:
    
                # make sure the EE is in the trajectory
                if not at.trajectory_has_ee(request.trajectory_name, ee): 
                    rospy.logwarn(str("ServiceInterface::handle_execute_command() -- " + ee + " not in trajectory, can't execute"))
                    continue

                # if the AT has prevously computed a valid plan (can't execute unless this is True) 
                if at.plan_to_waypoint_valid(str(ee),request.trajectory_name) :
                    # command the execution (this is currently open loop on the AT side -- FIXME)
                    r[ee] = at.move_to_waypoint(str(ee)) 
                    rospy.loginfo(str("ServiceInterface::handle_execute_command() -- done executing plan for " + ee))
                else :
                    rospy.loginfo(str("ServiceInterface::handle_execute_command() -- no valid plan found for " + ee))
     
            # status will return True only when all ee's in plan are valid
            response.status = (not False in r.values()) 
            response.affordance_template_status = self.get_template_status(request.type, request.id, request.trajectory_name)
    
        except:
            rospy.logerr(str("ServiceInterface::handle_execute_command() -- error performing command!!"))

        return response


    def handle_save_template(self, request):
        rospy.loginfo(str("ServiceInterface::handle_save_template() -- save request for robot " + request.original_class_type + ":" + str(request.id) + 
            " as " + request.new_class_type + ":" + str(request.id) + " with " + request.filename + " with image: " + request.image))
        response = SaveAffordanceTemplateResponse()
        response.status = False    
        try:
            new_key = str(request.new_class_type) + ":" + str(request.id)
            save_status = self.server.at_data.class_map[request.original_class_type][request.id].save_to_disk(filename=request.filename, image=request.image, new_class_type=new_key, save_scaling=request.save_scale_updates)
            remove_status = self.server.remove_template(request.original_class_type, request.id)
            self.server.at_data = self.server.get_available_templates(self.server.template_path, self.server.at_data)
            add_status = self.server.add_template(request.new_class_type, request.id)
            response.status = save_status and remove_status and add_status
        except:
            rospy.logerr("ServiceInterface::handle_load_robot()  -- Error trying to save template")
        return response

    def handle_add_trajectory(self, request):
        rospy.loginfo(str("ServiceInterface::handle_add_trajectory() -- add trajectory [" + request.trajectory_name +  "] to " + request.class_type + ":" + str(request.id)))
        response = AddAffordanceTemplateTrajectoryResponse()
        response.status = False    
        try:
            key = str(request.class_type) + ":" + str(request.id)
            self.server.at_data.class_map[request.class_type][request.id].add_trajectory(request.trajectory_name)
            response.status = True
        except:
            rospy.logerr("ServiceInterface::handle_add_trajectory()  -- Error trying to add trajectory")
        return response

    def handle_object_scale(self, request):
        rospy.loginfo(str("ServiceInterface::handle_object_scale() -- scale " + request.scale_info.class_type + ":" + str(request.scale_info.id) 
            + " -> object[" + request.scale_info.object_name + "] by " + str(request.scale_info.scale_factor) + "," + str(request.scale_info.end_effector_scale_factor) + ")"))
        response = ScaleDisplayObjectResponse()
        response.status = False    
        try:
            key = str(request.scale_info.class_type) + ":" + str(request.scale_info.id)
            self.server.at_data.class_map[request.scale_info.class_type][request.scale_info.id].scale_object(request.scale_info.object_name, request.scale_info.scale_factor, request.scale_info.end_effector_scale_factor)
            response.status = True
        except:
            rospy.logerr("ServiceInterface::handle_object_scale()  -- Error trying to scale object")
        return response


    def handle_object_scale_stream(self, data):
        rospy.loginfo(str("ServiceInterface::handle_object_scale_stream() -- scale " + data.class_type + ":" + str(data.id) 
            + " -> object[" + data.object_name + "] by (" + str(data.scale_factor) + "," + str(data.end_effector_scale_factor) + ")"))
        try:
            key = str(data.class_type) + ":" + str(data.id)
            self.server.at_data.class_map[data.class_type][data.id].scale_object(data.object_name, data.scale_factor, data.end_effector_scale_factor)
        except:
            rospy.logerr("ServiceInterface::handle_object_scale_stream()  -- Error trying to scale object")
        return


    def handle_template_status_request(self, request):
        response = GetAffordanceTemplateStatusResponse()
        response.affordance_template_status = []
        if request.name :
            try:
                ss = request.name.split(":")
                ats = self.get_template_status(ss[0], int(ss[1]), request.trajectory_name)
                response.affordance_template_status.append(ats)
                response.current_trajectory = self.server.at_data.class_map[ss[0]][int(ss[1])].current_trajectory
            except :
                rospy.logerr("ServiceInterface::handle_template_status_request() -- error getting template status")
        else :           
            rospy.logerr("ServiceInterface::handle_template_status_request() -- no template name provided")

        return response

    def get_template_status(self, template_name, template_id, trajectory_name) :
        ats = AffordanceTemplateStatus()
        try :
            # print template_name
            # print template_id
            # print trajectory_name
            at = self.server.at_data.class_map[template_name][template_id]
            ats.type = template_name
            ats.id = template_id
            if not trajectory_name : trajectory_name = at.current_trajectory
            ats.trajectory_name = trajectory_name
            ats.waypoint_info = []

            if not trajectory_name in at.waypoint_max.keys() :
                rospy.logerr(str("ServiceInterface::get_template_status() -- no trajectory caleld " + trajectory_name + " found"))
                return None

            for ee in at.robot_interface.end_effector_names :
                wp = WaypointInfo()
                wp.end_effector_name = ee
                wp.id = at.robot_interface.get_end_effector_id(ee)
                if not wp.id in at.waypoint_max[trajectory_name].keys() : continue
                wp.num_waypoints = at.waypoint_max[trajectory_name][wp.id]+1
                wp.waypoint_index = int(at.waypoint_index[trajectory_name][wp.id])
                wp.plan_valid = at.waypoint_plan_valid[trajectory_name][wp.id]
                wp.execution_valid = at.waypoint_execution_valid[trajectory_name][wp.id]  
                wp.waypoint_plan_index = at.waypoint_plan_index[trajectory_name][wp.id]                               
                ats.waypoint_info.append(wp)                
        except :
            rospy.logerr("ServiceInterface::get_template_status() -- error generating status message")
        return ats
