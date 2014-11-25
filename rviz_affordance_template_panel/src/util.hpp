#ifndef UTIL_HPP
#define UTIL_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <assert.h>
#include <iostream>
#include <sstream>
#include <string>
#include <kdl/frames.hpp>

#include <geometry_msgs/Pose.h>

namespace util
{

    std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);
    std::vector<std::string> split(const std::string &s, char delim);

    std::vector<float> quaternionToRPY(float x, float y, float z, float w);
    std::vector<float> RPYToQuaternion(float rr, float rp, float ry);

    std::vector<float> poseMsgToVector(geometry_msgs::Pose msg);
	geometry_msgs::Pose vectorToPoseMsg(std::vector<float> pose);

    std::string resolvePackagePath(const std::string& str);
 
}

#endif // UTIL_HPP
