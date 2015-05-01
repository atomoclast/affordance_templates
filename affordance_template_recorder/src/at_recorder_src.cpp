#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//#include <tf/tfScalar.h>
#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
  //std::cout << argc << std::endl;
  if (argc!=3)
  {
    std::cout << "Three arguments are needed" << std::endl;
    std::cout << "When running exe, the format should be \"exe <base-frame> <target-frame>\"" << std::endl;
    std::cout << "e.g. ./at_recorder Valve:0 r_default_tool" << std::endl;
    std::cout << "Quitting!" << std::endl;

    exit(1);
  }
  
  std::string base_frame = argv[1];
  std::string target_frame = argv[2];

  std::ofstream op_file_id;
  op_file_id.open("op_file.txt");

  //std::cout << base_frame << "\t" << target_frame << std::endl;
  
  ros::init(argc, argv, "record_traj_point_node");
  
  ros::NodeHandle node;

  ros::Rate rate(0.5);
 
  tf::StampedTransform transform;
  geometry_msgs::Vector3 position;
  tf::Matrix3x3 matrix;
  double roll, pitch, yaw;

  tf::TransformListener listener;

  while (node.ok())
  {

    try {
      listener.waitForTransform(base_frame, target_frame, ros::Time::now(), ros::Duration(3.0));
      listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
       ros::Duration(1.0).sleep();
       continue;
    }
    
    position.x = transform.getOrigin().x();
    position.y = transform.getOrigin().y();
    position.z = transform.getOrigin().z();

    matrix.setRotation(transform.getRotation());
    matrix.getRPY(roll, pitch, yaw);
    //matrix.getEulerYPR(yaw, pitch, roll);
    
    std::cout << position.x << "\t" << position.y << "\t" << position.z << "\t" << std::endl;
    std::cout << roll << "\t" << pitch << "\t" << yaw << "\t" << std::endl;
    
    op_file_id << position.x << "\t" << position.y << "\t" << position.z << "\n";
    op_file_id << roll << "\t" << pitch << "\t" << yaw << "\n\n";

    rate.sleep();

  }

  op_file_id.close();
  
  return 0;

}



