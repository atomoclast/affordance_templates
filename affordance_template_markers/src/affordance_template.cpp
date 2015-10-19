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
  loop_rate_(10.0)
{

  boost::thread at_thread(boost::bind(&AffordanceTemplate::run, this));

  setupMenuOptions();

  ROS_INFO("AffordanceTemplate::init() -- Done Creating new Empty AffordanceTemplate");

}


AffordanceTemplate::~AffordanceTemplate() 
{
	
}

void AffordanceTemplate::run()
{

  ros::AsyncSpinner spinner(1.0/loop_rate_);
  spinner.start();

  ROS_INFO("%s spinning.", nh_.getNamespace().c_str());
  ros::Rate loop_rate(loop_rate_);
  while(ros::ok())
  {
    loop_rate.sleep();
   // ROS_INFO("%s spinning.", nh_.getNamespace().c_str());

    //self.mutex.acquire()
    for(auto &f: frame_store_) {
      FrameInfo fi = f.second;
      tf::Transform transform;
      tf::poseMsgToTF(fi.second.pose, transform);
      tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), fi.second.header.frame_id, fi.first));

      // std::cout << "root frame: " << fi.second.header.frame_id << std::endl;
      // std::cout << "obj frame:  " << fi.first << std::endl;
      // std::cout << "pose: " << fi.second.pose << std::endl;
    }
  }
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

  initial_structure_ = structure;
  appendIDToStructure(structure);
  // self.load_initial_parameters(pose)
  createFromStructure(initial_structure_);

  // stuff = filename.split("/")
  // self.filename = stuff[len(stuff)-1]
  // return self.structure
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


    geometry_msgs::Pose p = robot_interface_->getRobotConfig().root_offset;

    tf::poseMsgToTF(p, robotTroot_);
    geometry_msgs::PoseStamped ps;
    ps.pose = p;
    ps.header.frame_id = robot_interface_->getRobotConfig().frame_id;
    //  frame_store_[template_type_] = std::make_pair<key_, ps>;
    frame_store_[key_] = FrameInfo(key_, ps);

    std::cout << "adding frame info for " << robot_interface_->getRobotConfig().frame_id << std::endl;
  }
//   self.frame_store_map[self.name] = FrameStore(self.key, self.robot_interface.robot_config.frame_id, getPoseFromFrame(self.robotTroot))
//   if not current_trajectory :
//       for traj in self.structure['end_effector_trajectory'] :
//           if self.is_valid_trajectory(traj) :
//               self.current_trajectory = str(traj['name'])
//               rospy.loginfo("AffordanceTemplate::create_from_parameters() -- setting current trajectory to: " + str(traj['name']))
//               break

//   # parse objects
//   ids = 0
//   debug_id = 0

  int idx = 0;

  for(auto &obj: structure.display_objects) {

    setupObjectMenu(obj);

    std::string root_frame = template_type_;

    // if(getRootObject() == obj.name) {
    //   root_frame = key_;
    // } else {
    //   if(obj.parent != "") {
    //     root_frame = obj.parent;
    //   }
    // }

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
    int_marker.header.frame_id = "/" + root_frame;
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
    // if(self.object_controls_display_on) :
    //     int_marker.controls.extend(CreateCustomDOFControls("",
    //         self.object_controls[obj]['xyz'][0], self.object_controls[obj]['xyz'][1], self.object_controls[obj]['xyz'][2],
    //         self.object_controls[obj]['rpy'][0], self.object_controls[obj]['rpy'][1], self.object_controls[obj]['rpy'][2]))


    //int_markers_[obj.name] = control.markers[0];// i think this is wrong
    //self.marker_pose_offset[obj] = self.pose_from_origin(self.object_origin[obj])

    // if self.object_controls_display_on :
    //     self.marker_menus[obj].setCheckState( self.menu_handles[(obj,"Hide Controls")], MenuHandler.UNCHECKED )
    // else :
    //     self.marker_menus[obj].setCheckState( self.menu_handles[(obj,"Hide Controls")], MenuHandler.CHECKED )

    addInteractiveMarker(int_marker);
    // self.marker_menus[obj].apply( self.server, obj )
    server_->applyChanges();

    idx += 1;



  }

  return true;
}


void AffordanceTemplate::setupObjectMenu(DisplayObject obj)
{

}

void AffordanceTemplate::setupWaypointMenu(EndEffector ee)
{

}


void AffordanceTemplate::addInteractiveMarker(visualization_msgs::InteractiveMarker m)
{
  std::string name = m.name;
  ROS_INFO("AffordanceTemplate::addInteractiveMarker() -- %s", m.name.c_str());
  int_markers_[m.name] = m;
  server_->insert(m);
  server_->setCallback(m.name, boost::bind( &AffordanceTemplate::processFeedback, this, _1 ));

  std::cout << m << std::endl;
}



void AffordanceTemplate::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback ) 
{
  ROS_INFO("AffordanceTemplate::processFeedback() -- %s", feedback->marker_name.c_str());
}


bool AffordanceTemplate::hasObjectFrame(std::string obj) {
  return (frame_store_.find(obj) != std::end(frame_store_)); 
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


