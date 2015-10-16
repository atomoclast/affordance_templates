
#include <affordance_template_library/affordance_template_parser.h>

using namespace affordance_template_object;

AffordanceTemplateParser::AffordanceTemplateParser() {}
AffordanceTemplateParser::~AffordanceTemplateParser() {}

bool AffordanceTemplateParser::loadFromFile(std::string filename, AffordanceTemplateStructure &at)
{

  char *cstr = new char[filename.length() + 1];
  strcpy(cstr, filename.c_str());
  char* str = std::strtok(cstr, "/");

  while(std::string(str).find(".json") == std::string::npos && ros::ok())
      str = std::strtok(NULL, "/");

  std::string template_name(str);
  ROS_INFO("[AffordanceTemplateParser::loadFromFile] Found template name: %s", template_name.c_str());
  
  delete [] cstr;


  FILE* f_pnt = std::fopen(filename.c_str(), "r");
  char json_buff[65536];
  rapidjson::FileReadStream json(f_pnt, json_buff, sizeof(json_buff));

  rapidjson::Document jdoc;
  if (jdoc.ParseStream(json).HasParseError())
  {
      ROS_WARN("[AffordanceTemplateParser::loadFromFile] couldn't properly parse template; ignoring.");
  }
  else
  {
      ROS_INFO("[AffordanceTemplateParser::loadFromFile] parsing template: %s", template_name.c_str());

      at.name = jdoc["name"].GetString();
      ROS_INFO_STREAM("[AffordanceTemplateParser::loadFromFile] name is "<<at.name);
      at.image = jdoc["image"].GetString();
      ROS_INFO_STREAM("[AffordanceTemplateParser::loadFromFile] img is "<<at.image);
      at.filename = filename;
      ROS_INFO_STREAM("[AffordanceTemplateParser::loadFromFile] filename is "<<at.filename);

      const rapidjson::Value& traj = jdoc["end_effector_trajectory"];
      for (rapidjson::SizeType t = 0; t < traj.Size(); ++t)
      {
          EndEffector ee;
          ee.name = traj[t]["name"].GetString();
          const rapidjson::Value& ee_group = traj[t]["end_effector_group"];
          ee.id = ee_group[0]["id"].GetInt();
          ROS_INFO_STREAM("[AffordanceTemplateParser::loadFromFile] found EE trajectory with name: "<<ee.name<<" and id: "<<ee.id);

          const rapidjson::Value& waypoints = ee_group[0]["end_effector_waypoint"];
          for (rapidjson::SizeType w = 0; w < waypoints.Size(); ++w)
          {
              // find the origin
              Origin org;
              const rapidjson::Value& pos = waypoints[w]["origin"]["xyz"];
              org.position[0] = pos[0].GetDouble();
              org.position[1] = pos[1].GetDouble();
              org.position[2] = pos[2].GetDouble();
              const rapidjson::Value& euler = waypoints[w]["origin"]["rpy"];
              org.orientation[0] = euler[0].GetDouble();
              org.orientation[1] = euler[1].GetDouble();
              org.orientation[2] = euler[2].GetDouble();

              // create the controls
              // ** note: it's a little cryptic but basically some of the jsons have "1" for true under controls
              // **       so this checks to see if it's bool or int, then converts int if not bool
              Control ctrl;
              const rapidjson::Value& trans = waypoints[w]["controls"]["xyz"];
              ctrl.translation[0] = trans[0].IsBool() ? trans[0].GetBool() : (trans[0].GetInt() == 1 ? true : false);
              ctrl.translation[1] = trans[1].IsBool() ? trans[1].GetBool() : (trans[1].GetInt() == 1 ? true : false);
              ctrl.translation[2] = trans[2].IsBool() ? trans[2].GetBool() : (trans[2].GetInt() == 1 ? true : false);
              const rapidjson::Value& rot = waypoints[w]["controls"]["rpy"];
              ctrl.rotation[0] = rot[0].IsBool() ? rot[0].GetBool() : (rot[0].GetInt() == 1 ? true : false);
              ctrl.rotation[1] = rot[1].IsBool() ? rot[1].GetBool() : (rot[1].GetInt() == 1 ? true : false);
              ctrl.rotation[2] = rot[2].IsBool() ? rot[2].GetBool() : (rot[2].GetInt() == 1 ? true : false);
              ctrl.scale = waypoints[w]["controls"]["scale"].GetDouble();

              // fill out the waypoint
              Waypoint wp;
              wp.ee_pose = waypoints[w]["ee_pose"].GetInt();
              wp.display_object = waypoints[w]["display_object"].GetString();
              wp.origin = org;
              wp.controls = ctrl;

              ee.waypoints.push_back(wp);
              
              ROS_INFO_STREAM("[AffordanceTemplateParser::loadFromFile]     waypoint "<<w+1<<" has ee_pose: "<<wp.ee_pose<<" and display_object: "<<wp.display_object);
              ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tat origin XYZ: %g %g %g and RPY: %g %g %g", org.position[0], org.position[1], org.position[2], org.orientation[0], org.orientation[1], org.orientation[2]);
              ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: XYZ: %s %s %s", ctrl.toBoolString(ctrl.translation[0]).c_str(), ctrl.toBoolString(ctrl.translation[1]).c_str(), ctrl.toBoolString(ctrl.translation[2]).c_str());
              ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: RPY: %s %s %s", ctrl.toBoolString(ctrl.rotation[0]).c_str(), ctrl.toBoolString(ctrl.rotation[1]).c_str(), ctrl.toBoolString(ctrl.rotation[2]).c_str());
          }

          at.ee_trajectories.push_back(ee);
      }

      const rapidjson::Value& objects = jdoc["display_objects"];
      for (rapidjson::SizeType i = 0; i < objects.Size(); ++i)
      {
          // find the origin
          Origin orig;
          const rapidjson::Value& pos = objects[i]["origin"]["xyz"];
          orig.position[0] = pos[0].GetDouble();
          orig.position[1] = pos[1].GetDouble();
          orig.position[2] = pos[2].GetDouble();
          const rapidjson::Value& euler = objects[i]["origin"]["rpy"];
          orig.orientation[0] = euler[0].GetDouble();
          orig.orientation[1] = euler[1].GetDouble();
          orig.orientation[2] = euler[2].GetDouble();

          // create the controls
          // ** note: it's a little cryptic but basically some of the jsons have "1" for true under controls
          // **       so this checks to see if it's bool or int, then converts int if not bool
          Control ctrl;
          const rapidjson::Value& trans = objects[i]["controls"]["xyz"];
          ctrl.translation[0] = trans[0].IsBool() ? trans[0].GetBool() : (trans[0].GetInt() == 1 ? true : false);
          ctrl.translation[1] = trans[1].IsBool() ? trans[1].GetBool() : (trans[1].GetInt() == 1 ? true : false);
          ctrl.translation[2] = trans[2].IsBool() ? trans[2].GetBool() : (trans[2].GetInt() == 1 ? true : false);
          const rapidjson::Value& rot = objects[i]["controls"]["rpy"];
          ctrl.rotation[0] = rot[0].IsBool() ? rot[0].GetBool() : (rot[0].GetInt() == 1 ? true : false);
          ctrl.rotation[1] = rot[1].IsBool() ? rot[1].GetBool() : (rot[1].GetInt() == 1 ? true : false);
          ctrl.rotation[2] = rot[2].IsBool() ? rot[2].GetBool() : (rot[2].GetInt() == 1 ? true : false);
          ctrl.scale = objects[i]["controls"]["scale"].GetDouble();

          Shape shp;
          shp.type = objects[i]["shape"]["type"].GetString();
          std::string shape_str = "shape of type: ";
          if (shp.type == "mesh")
          {
              shp.mesh = objects[i]["shape"]["data"].GetString();
              shape_str += "mesh using mesh file: " + shp.mesh;
          }
          else // some other shape which will have material
          {
              shp.color = objects[i]["shape"]["material"]["color"].GetString();
              const rapidjson::Value& color = objects[i]["shape"]["material"]["rgba"];
              shp.rgba[0] = color[0].GetDouble();
              shp.rgba[1] = color[1].GetDouble();
              shp.rgba[2] = color[2].GetDouble();
              shp.rgba[3] = color[3].GetDouble();

              std::ostringstream rgba;
              rgba << shp.rgba[0] << " " << shp.rgba[1] << " " << shp.rgba[2] << " " << shp.rgba[3];
              shape_str += shp.type + " colored: " + shp.color + " using RGBA: " + rgba.str();
          }
          const rapidjson::Value& sz = objects[i]["shape"]["size"];
          shp.size[0] = sz[0].GetDouble();
          shp.size[1] = sz[1].GetDouble();
          shp.size[2] = sz[2].GetDouble();

          DisplayObject display_object;
          display_object.name = objects[i]["name"].GetString();

          if(objects[i].HasMember("parent")) {  
            display_object.parent = objects[i]["parent"].GetString();
          } else {
            display_object.parent = "";
          }
          display_object.origin = orig;
          display_object.shape = shp;
          display_object.controls = ctrl;

          ROS_INFO_STREAM("[AffordanceTemplateParser::loadFromFile] disp  lay object "<<i+1<<" has name: "<<display_object.name);
          if(display_object.parent != "") {
            ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tparent object: %s", display_object.parent.c_str());
          }
          ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tat origin XYZ: %g %g %g and RPY: %g %g %g", orig.position[0], orig.position[1], orig.position[2], orig.orientation[0], orig.orientation[1], orig.orientation[2]);
          ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: XYZ: %s %s %s", ctrl.toBoolString(ctrl.translation[0]).c_str(), ctrl.toBoolString(ctrl.translation[1]).c_str(), ctrl.toBoolString(ctrl.translation[2]).c_str());
          ROS_INFO("[AffordanceTemplateParser::loadFromFile] \tcontrol for axes set to: RPY: %s %s %s", ctrl.toBoolString(ctrl.rotation[0]).c_str(), ctrl.toBoolString(ctrl.rotation[1]).c_str(), ctrl.toBoolString(ctrl.rotation[2]).c_str());
          ROS_INFO("[AffordanceTemplateParser::loadFromFile] \t%s", shape_str.c_str());

          at.display_objects.push_back(display_object);
      }

      std::cout<<std::endl;

      // at_collection_[at.name] = at;
  }
  std::fclose(f_pnt);

  return true;
}