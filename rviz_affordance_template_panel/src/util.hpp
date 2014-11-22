#ifndef UTIL_HPP
#define UTIL_HPP

#include <ros/ros.h>
#include <ros/package.h>

#include <vector>
#include <assert>
#include <iostream>
#include <sstream>
#include <string>
#include <kdl/frames.hpp>

#include <geometry_msgs/Pose.h>

using namespace std;

namespace util
{

    vector<string> &split(const string &s, char delim, vector<string> &elems);
    vector<string> split(const string &s, char delim);

    vector<float> quaternionToRPY(float x, float y, float z, float w);
    vector<float> RPYToQuaternion(float rr, float rp, float ry);

    vector<float> poseMsgToVector(geometry_msgs::Pose msg);
	geometry_msgs::Pose vectorToPoseMsg(vector<float> pose);

    string resolvePackagePath(const string& str);

}

#endif // UTIL_HPP
