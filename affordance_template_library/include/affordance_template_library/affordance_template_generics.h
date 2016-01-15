#ifndef _AT_GENERICS_H_
#define _AT_GENERICS_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>
#include <planner_interface/planner_interface.h>

namespace affordance_template_object
{
    struct Origin
    {
        double position[3]; // xyz
        double orientation[3]; // rpy
    };

    struct ToleranceBounds
    {
        double position[3][2]; // xyz [lo,hi]
        double orientation[3][2]; // rpy [lo,hi]
    };

    struct TaskCompatibility
    {
        int position[3]; // xyz
        int orientation[3]; // rpy
    };

    inline geometry_msgs::Pose originToPoseMsg(Origin origin) 
    { 
      geometry_msgs::Pose p;
      p.position.x = origin.position[0];
      p.position.y = origin.position[1];
      p.position.z = origin.position[2];
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(origin.orientation[0],origin.orientation[1],origin.orientation[2]);
      return p;
    }

    inline geometry_msgs::Pose taskCompatibilityToPoseMsg(TaskCompatibility tc) 
    { 
      geometry_msgs::Pose p;
      p.position.x = tc.position[0];
      p.position.y = tc.position[1];
      p.position.z = tc.position[2];
      p.orientation = tf::createQuaternionMsgFromRollPitchYaw(tc.orientation[0],tc.orientation[1],tc.orientation[2]);
      return p;
    }

    inline planner_interface::PlannerType stringToPlannerType(std::string planner_type) {
        if(planner_type == "CARTESIAN") {
            return planner_interface::CARTESIAN;
        } else {
            return planner_interface::JOINT;  
        }
    }    

    struct Shape
    {
        std::string color;
        double rgba[4];
        std::string type;
        std::string mesh; // if type == "mesh" then we need to fill the file name in here
        double size[3]; // for boxes and meshes
        double length; // for cylinders
        double radius; // for spherse and cylinders
    };

    inline std::string toBoolString(bool b) { return (b ? "true" : "false"); }

    struct Control
    {
        bool translation[3]; // xyz
        bool rotation[3]; // rpy
        double scale;
    };
}

#endif