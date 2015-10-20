#include <affordance_template_markers/affordance_template.h>

using namespace affordance_template;
using namespace affordance_template_object;
using namespace affordance_template_markers;

AffordanceTemplate::AffordanceTemplate(const ros::NodeHandle nh, 
                                        boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server,  
                                        std::string robot_name, 
                                        std::string template_type,
                                        int id) :
  nh_(nh),
  server_(server),
  robot_name_(robot_name),
  template_type_(template_type),
  id_(id),
  root_object_(""),
  loop_rate_(10.0),
  object_controls_display_on_(true)
{

  setupMenuOptions();

  ROS_INFO("AffordanceTemplate::init() -- Done Creating new AffordanceTemplate of type %s for robot: %s", template_type_.c_str(), robot_name_.c_str());

}


AffordanceTemplate::~AffordanceTemplate() 
{
	
}

void AffordanceTemplate::setRobotInterface(boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface)
{
  robot_interface_ = robot_interface;
}


void AffordanceTemplate::setupMenuOptions() 
{

  waypoint_menu_options_.clear();
  waypoint_menu_options_.push_back(MenuConfig("Change End-Effector Pose", false));
  waypoint_menu_options_.push_back(MenuConfig("Hide Controls", true));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  waypoint_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  waypoint_menu_options_.push_back(MenuConfig("Delete Waypoint", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Forward", false));
  waypoint_menu_options_.push_back(MenuConfig("Move Back", false));

  object_menu_options_.clear();
  object_menu_options_.push_back(MenuConfig("Add Waypoint Before", false));
  object_menu_options_.push_back(MenuConfig("Add Waypoint After", false));
  object_menu_options_.push_back(MenuConfig("Reset", false));
  object_menu_options_.push_back(MenuConfig("Save", false));
  object_menu_options_.push_back(MenuConfig("Hide Controls", true));
  object_menu_options_.push_back(MenuConfig("Choose Trajectory", false));

}

bool AffordanceTemplate::loadFromFile(std::string filename, geometry_msgs::Pose pose, AffordanceTemplateStructure &structure)
{

  at_parser_.loadFromFile(filename, structure);

  // store copies in class, one as backup to reset/restore later
  initial_structure_ = structure;
  structure_ = structure;

  appendIDToStructure(structure_);
  createFromStructure(structure_);

  return true;
}

bool AffordanceTemplate::appendIDToStructure(AffordanceTemplateStructure &structure) 
{

  structure.name = appendID(structure.name);
  for(auto &obj : structure.display_objects) {
      obj.name = appendID(obj.name);
      if(obj.parent != "") {
        obj.parent = appendID(obj.parent);
      }
  }

  //       for traj in structure['end_effector_trajectory'] :
  //           for ee_group in traj['end_effector_group'] :
  //               for wp in ee_group['end_effector_waypoint'] :
  //                   wp['display_object'] = self.append_id(wp['display_object'])

  return true;

}
 
std::string AffordanceTemplate::appendID(std::string s) 
{
  return s + ":" + std::to_string(id_);
} 


bool AffordanceTemplate::createFromStructure(AffordanceTemplateStructure structure, bool keep_poses, std::string traj) 
{

  ROS_INFO("AffordanceTemplate::createFromStructure() -- %s", template_type_.c_str());

  key_ = structure.name;

  {
    geometry_msgs::PoseStamped ps;
    ps.pose = robot_interface_->getRobotConfig().root_offset;
    ps.header.frame_id = robot_interface_->getRobotConfig().frame_id;
    frame_store_[key_] = FrameInfo(key_, ps);
  }

  // set the default traj as the first valid one found
  if(current_trajectory_=="") {
    for (auto &t: structure.ee_trajectories) {
      if(isValidTrajectory(t)) {
        current_trajectory_ = t.name;
        ROS_INFO("AffordanceTemplate::createFromStructure() -- setting current trajectory to: %s", current_trajectory_.c_str());
        break;
      }
    }
  }

  int idx = 0;

  for(auto &obj: structure.display_objects) {

    setupObjectMenu(structure, obj);
    std::string root_frame = template_type_;

    if(obj.parent != "") {
      root_frame = obj.parent;
    } else {
      root_frame = key_;
    }

    // set scale factor if not already set
    if(object_scale_factor_.find(obj.name) == std::end(object_scale_factor_)) {
      object_scale_factor_[obj.name] = 1.0;
    }

    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = root_frame;
    int_marker.header.stamp = ros::Time::now();
    int_marker.name = obj.name;
    int_marker.description = obj.name;
    int_marker.scale = obj.controls.scale*object_scale_factor_[obj.name];

    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;    

    visualization_msgs::Marker marker;
    marker.ns = obj.name;
    marker.id = idx;    
  
    if(!keep_poses || !hasObjectFrame(obj.name) ) {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = root_frame;
      ps.pose = originToPoseMsg(obj.origin);
      frame_store_[obj.name] = FrameInfo(obj.name, ps);
      int_marker.pose = ps.pose;
    } else {
      int_marker.pose = frame_store_[obj.name].second.pose;
      if(obj.parent != "") {
        int_marker.pose.position.x /= object_scale_factor_[obj.name];
        int_marker.pose.position.y /= object_scale_factor_[obj.name];
        int_marker.pose.position.z /= object_scale_factor_[obj.name];
      }
    }

    // adjust for scale factor of parent object
    if(obj.parent != "") {
      int_marker.pose.position.x *= object_scale_factor_[obj.name];
      int_marker.pose.position.y *= object_scale_factor_[obj.name];
      int_marker.pose.position.z *= object_scale_factor_[obj.name];
    }
    frame_store_[obj.name].second.pose = int_marker.pose;

    if(obj.shape.type == "mesh") {
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      marker.mesh_resource = obj.shape.mesh;
      marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
      marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
      ROS_INFO("Drawing Mesh for object %s : %s (scale=%.3f)", obj.name.c_str(), marker.mesh_resource.c_str(), object_scale_factor_[obj.name]);
    } else if(obj.shape.type == "box") {
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
      marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
    } else if(obj.shape.type == "sphere") {
      marker.type = visualization_msgs::Marker::SPHERE;
      marker.scale.x = obj.shape.size[0]*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.size[1]*object_scale_factor_[obj.name];
      marker.scale.z = obj.shape.size[2]*object_scale_factor_[obj.name];
    } else if(obj.shape.type == "cylinder") {
      marker.type = visualization_msgs::Marker::CYLINDER;
      marker.scale.x = obj.shape.radius*object_scale_factor_[obj.name];
      marker.scale.y = obj.shape.length*object_scale_factor_[obj.name];      
    }

    control.markers.push_back(marker);

    if(obj.shape.type != "mesh") {
      control.markers[0].color.r = obj.shape.rgba[0];
      control.markers[0].color.g = obj.shape.rgba[1];
      control.markers[0].color.b = obj.shape.rgba[2];
      control.markers[0].color.a = obj.shape.rgba[3];
    } else {
      control.markers[0].mesh_use_embedded_materials = false;
      control.markers[0].color.r = 1.0;
      control.markers[0].color.g = 1.0;
      control.markers[0].color.b = 1.0;
      control.markers[0].color.a = 1.0;
    }

    // double scale = 1.0;
    // if obj in self.object_controls :
    //     scale = self.object_controls[obj]['scale']*self.object_scale_factor[obj]

    //  # int_marker = CreateInteractiveMarker(self.frame_id, obj.name, scale)
    int_marker.controls.push_back(control);
    if(object_controls_display_on_) {
      std::vector<visualization_msgs::InteractiveMarkerControl> dof_controls;
      dof_controls = utils::MarkerHelper::makeCustomDOFControls(obj.controls.translation[0], 
                                                                obj.controls.translation[1], 
                                                                obj.controls.translation[2],
                                                                obj.controls.rotation[0],
                                                                obj.controls.rotation[1],
                                                                obj.controls.rotation[2]);

      for (auto &c: dof_controls) {
        int_marker.controls.push_back(c);
      }
    }   

    //self.marker_pose_offset[obj] = self.pose_from_origin(self.object_origin[obj])

    addInteractiveMarker(int_marker);

    MenuHandleKey key;
    key[obj.name] = {"Hide Controls"};
    if(object_controls_display_on_) {
      marker_menus_[obj.name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
    } else {
      marker_menus_[obj.name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
    }

    marker_menus_[obj.name].apply( *server_, obj.name );
    server_->applyChanges();

    idx += 1;

  }

  return true;
}


bool AffordanceTemplate::isValidTrajectory(Trajectory traj)  
{
  bool valid_trajectory = true;
  for(auto &g: traj.end_effector_waypoint_list) {
    for(auto &wp: g.waypoints) {
      if(robot_interface_->getEENameMap().find(g.id) == std::end(robot_interface_->getEENameMap())) {
        valid_trajectory = false;
        ROS_DEBUG("AffordanceTemplate::is_valid_trajectory() -- can't find ee ID: %d", g.id);
        return valid_trajectory;
      }
   }
 }
 return valid_trajectory;
} 
  


void AffordanceTemplate::setupObjectMenu(AffordanceTemplateStructure structure, DisplayObject obj)
{

  for(auto& o : object_menu_options_) {
    if(o.first == "Choose Trajectory") {
      setupTrajectoryMenu(structure, obj.name);
    } else {
      setupSimpleMenuItem(structure, obj.name, o.first, o.second);
    }
  }


  // for m,c in self.object_menu_options :
  //     if m == "Add Waypoint Before" or m == "Add Waypoint After":
  //         sub_menu_handle = self.marker_menus[obj].insert(m)
  //         for ee in self.robot_interface.end_effector_name_map.iterkeys() :
  //             name = self.robot_interface.end_effector_name_map[ee]
  //             self.menu_handles[(obj,m,name)] = self.marker_menus[obj].insert(name,parent=sub_menu_handle,callback=self.create_waypoint_callback)


}

void AffordanceTemplate::setupWaypointMenu(AffordanceTemplateStructure structure, EndEffectorWaypointList ee)
{

}

void AffordanceTemplate::setupSimpleMenuItem(AffordanceTemplateStructure structure, const std::string& name, const std::string& menu_text, bool has_check_box)
{
  MenuHandleKey key;
  key[name] = {menu_text};
  group_menu_handles_[key] = marker_menus_[name].insert( menu_text, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );
  if(has_check_box) {
    marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );
  }
}


void AffordanceTemplate::setupTrajectoryMenu(AffordanceTemplateStructure structure, const std::string& name)
{
  std::string menu_text = "Choose Trajectory";
  interactive_markers::MenuHandler::EntryHandle sub_menu_handle = marker_menus_[name].insert( menu_text );
  for(auto &traj: structure.ee_trajectories) {
    MenuHandleKey key;
    key[name] = {menu_text, traj.name};
    group_menu_handles_[key] = marker_menus_[name].insert( sub_menu_handle, traj.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ) );   
    if(traj.name == current_trajectory_) {
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::CHECKED );
    } else {
      marker_menus_[name].setCheckState( group_menu_handles_[key], interactive_markers::MenuHandler::UNCHECKED );       
    }
  }
}

void AffordanceTemplate::addInteractiveMarker(visualization_msgs::InteractiveMarker m)
{
  std::string name = m.name;
  ROS_INFO("AffordanceTemplate::addInteractiveMarker() -- %s with frame: %s", m.name.c_str(), m.header.frame_id.c_str());
  int_markers_[m.name] = m;
  server_->insert(m);
  server_->setCallback(m.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ));
}

void AffordanceTemplate::removeInteractiveMarker(std::string marker_name) 
{
  server_->erase(marker_name);
  server_->applyChanges();
}

void AffordanceTemplate::removeAllMarkers() {
  for(auto &m: int_markers_) {
    removeInteractiveMarker(m.first);
  }
  group_menu_handles_.clear();
  int_markers_.clear();
  marker_menus_.clear();
  server_->applyChanges();
}

void AffordanceTemplate::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) 
{

  ROS_INFO("AffordanceTemplate::processFeedback() -- %s", feedback->marker_name.c_str());

  interactive_markers::MenuHandler::CheckState state;

  // set up key maps for easy comparison to menu handler ID
  MenuHandleKey reset_key;
  MenuHandleKey save_key;
  MenuHandleKey hide_controls_key;
  
  reset_key[feedback->marker_name] = {"Reset"};
  save_key[feedback->marker_name] = {"Save"};
  hide_controls_key[feedback->marker_name] = {"Hide Controls"};

  if(hasObjectFrame(feedback->marker_name)) {
    geometry_msgs::Pose p = feedback->pose;
    if(feedback->header.frame_id != frame_store_[feedback->marker_name].second.header.frame_id) {
      geometry_msgs::PoseStamped ps;
      ps.pose = feedback->pose;
      ps.header = feedback->header;
      tf_listener_.transformPose (frame_store_[feedback->marker_name].second.header.frame_id, ps, ps);
      p = ps.pose;
    }
    frame_store_[feedback->marker_name].second.pose = p;
  }
  
  switch ( feedback->event_type ) {

    ROS_INFO("AffordanceTemplate::processFeedback() -- %s", feedback->marker_name.c_str());

    case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP :      
      ROS_DEBUG("AffordanceTemplate::processFeedback() --   MOUSE_UP");
      break;

    case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT :

      ROS_DEBUG("AffordanceTemplate::processFeedback() --   MENU_SELECT");
      ROS_DEBUG("AffordanceTemplate::processFeedback() --   menu id: %d", feedback->menu_entry_id);
      ROS_DEBUG("AffordanceTemplate::processFeedback() --   pose: (%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f), frame_id: %s", 
                                                                  feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z, 
                                                                  feedback->pose.orientation.x, feedback->pose.orientation.y, feedback->pose.orientation.z, feedback->pose.orientation.w,
                                                                  feedback->header.frame_id.c_str());

      if(group_menu_handles_.find(reset_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[reset_key] == feedback->menu_entry_id) {
          ROS_DEBUG("AffordanceTemplate::processFeedback() --   RESET");
          structure_ = initial_structure_;      
          appendIDToStructure(structure_);
          removeAllMarkers();
          createFromStructure(structure_, false, current_trajectory_); //FIXME:: make sure current_traj is i ORIGINAL list of traj
        }
      }

      if(group_menu_handles_.find(save_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[save_key] == feedback->menu_entry_id) {
        }
      }

      if(group_menu_handles_.find(hide_controls_key) != std::end(group_menu_handles_)) {
        if(group_menu_handles_[hide_controls_key] == feedback->menu_entry_id) {
          if(marker_menus_[feedback->marker_name].getCheckState( feedback->menu_entry_id, state ) ) {
            if(state == interactive_markers::MenuHandler::CHECKED) {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::CHECKED );
              object_controls_display_on_ = true;
            } else {
              marker_menus_[feedback->marker_name].setCheckState( feedback->menu_entry_id, interactive_markers::MenuHandler::UNCHECKED );
              object_controls_display_on_ = false;
            }
          }
          removeAllMarkers();
          createFromStructure(structure_, true, current_trajectory_);
        }
      }
      
      break;

    default :
      break;
  }
  server_->applyChanges();

}


bool AffordanceTemplate::hasObjectFrame(std::string obj) {
  return (frame_store_.find(obj) != std::end(frame_store_)); 
}


void AffordanceTemplate::run()
{
 
  ros::Rate loop_rate(loop_rate_);
  tf::Transform transform;
  FrameInfo fi;

  ros::AsyncSpinner spinner(1.0/loop_rate_);
  spinner.start();

  ROS_INFO("%s spinning.", nh_.getNamespace().c_str());
  while(ros::ok())
  {
    for(auto &f: frame_store_) {
      fi = f.second;
      tf::poseMsgToTF(fi.second.pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fi.second.header.frame_id, fi.first));
    }
    loop_rate.sleep();
  }

}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "affordance_template_test");
  ros::NodeHandle nh("~");
 
  std::string robot_name, template_type;
  
  if (nh.hasParam("robot_name")) {
    nh.getParam("robot_name", robot_name); 
  } else {
    robot_name = "r2_upperbody";
  }

  if (nh.hasParam("template_type")) {
    nh.getParam("template_type", template_type); 
  } else {
    template_type = "wheel";
  }

  boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface;
  robot_interface.reset(new affordance_template_markers::RobotInterface());
  robot_interface->load("r2_upperbody.yaml");

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
  server.reset( new interactive_markers::InteractiveMarkerServer(std::string(robot_name + "_affordance_template_server"),"",false) );

  AffordanceTemplate at(nh, server, robot_name, template_type, 0);
  at.setRobotInterface(robot_interface);
  
  AffordanceTemplateStructure structure;
  geometry_msgs::Pose p;
  at.loadFromFile("/home/swhart/ros/catkin_workspace/src/affordance_templates/affordance_template_library/templates/wheel.json", p, structure);

  at.run();
 
  return 0;
}


