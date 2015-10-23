#ifndef _AT_GENERICS_H_
#define _AT_GENERICS_H_

#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

namespace affordance_template_object
{
    struct Origin
    {
        double position[3]; // xyz
        double orientation[3]; // rpy
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

    struct Shape
    {
        std::string color;
        double rgba[4];
        std::string type;
        std::string mesh; // if type == "mesh" then we need to fill the file name in here
        double size[3]; // for cubes and meshes
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