#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//#include <tf/tfScalar.h>
#include <iostream>
#include <fstream>
#include <kdl/frames.hpp>

int main(int argc, char** argv)
{
  //std::cout << argc << std::endl;
  if (argc!=3)
  {
    std::cout << "Three arguments are needed" << std::endl;
    std::cout << "./at_recorder [obj_frame] [right|left]" << std::endl;
    std::cout << "Quitting!" << std::endl;

    exit(1);
  }
  
  std::string base_frame = argv[1];
  std::string hand = argv[2];
  std::string target_frame = "";

  if(hand=="right") {
    target_frame = "r_end";
  } else {
    target_frame = "l_end";
  }
  
  std::ofstream op_file_id;
  op_file_id.open("op_file.txt");

  KDL::Frame T_ee_obj, T_abs_obj, T_abs_ee;
  KDL::Frame T_ee_abs;
  KDL::Frame T_viz_offset;
  //T_ee_obj = T_abs_obj * T_ee_abs
  //T_abs_obj = T_ee_obj * inv(T_ee_abs)
  
  //std::cout << base_frame << "\t" << target_frame << std::endl;
  
  ros::init(argc, argv, "record_traj_point_node");
  ros::NodeHandle node;
  ros::Rate rate(0.1);
 
  tf::StampedTransform transform;
  geometry_msgs::Vector3 position;
  tf::Matrix3x3 tf_matrix;
  double roll, pitch, yaw;

  tf::TransformListener listener;

  while (node.ok())
  {

    try {
      listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
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

    tf_matrix.setRotation(transform.getRotation());
    tf_matrix.getRPY(roll, pitch, yaw);
    tf_matrix.getEulerYPR(yaw, pitch, roll);


    //Populate the translation elements of T_ee_obj
    T_ee_obj.p.data[0] = position.x;
    T_ee_obj.p.data[1] = position.y;
    T_ee_obj.p.data[2] = position.z;

    //Populate the rotation elements of T_ee_obj
    T_ee_obj.M = KDL::Rotation::RPY(roll, pitch, yaw);

    std::cout << "hand wrt object" << std::endl;
    std::cout << "position: " << position.x << "\t" << position.y << "\t" << position.z << "\t" << std::endl;
    std::cout << "rotation: " << roll << "\t" << pitch << "\t" << yaw << "\t" << std::endl << std::endl;
    
    /*op_file_id << position.x << "\t" << position.y << "\t" << position.z << "\n";
    op_file_id << roll << "\t" << pitch << "\t" << yaw << "\n\n";*/

    //Populate the translation elements of T_ee_abs
    if(hand=="right") {
      T_ee_abs.p.data[0] = 0.1;
      T_ee_abs.p.data[1] = -0.2;
      T_ee_abs.p.data[2] = 0.0;
      T_ee_abs.M = KDL::Rotation::RPY(0.0, 1.57, 3.14);
    } else {
      T_ee_abs.p.data[0] = 0.1;
      T_ee_abs.p.data[1] = 0.2;
      T_ee_abs.p.data[2] = 0.0;
      T_ee_abs.M = KDL::Rotation::RPY(0.0, 1.57, 3.14);
    }
    std::cout << "yaml: hand wrt abstract" << std::endl;
    std::cout << "position: " << T_ee_abs.p.data[0] << "\t" << T_ee_abs.p.data[1] << "\t" << T_ee_abs.p.data[2] << "\t" << std::endl;
    std::cout << "rotation: " << 0.0 << "\t" << 1.57 << "\t" << 3.14 << "\t" << std::endl << std::endl;


    if(hand=="right") {
      T_viz_offset.p.data[0] = 0.0;
      T_viz_offset.p.data[1] = -0.1;
      T_viz_offset.p.data[2] = 0.0;
      T_viz_offset.M = KDL::Rotation::RPY(0.0, 0.0, -1.57);
    } else {
      T_viz_offset.p.data[0] = 0.0;
      T_viz_offset.p.data[1] = 0.1;
      T_viz_offset.p.data[2] = 0.0;
      T_viz_offset.M = KDL::Rotation::RPY(0.0, 0.0, 1.57);
    }
    T_abs_obj = T_ee_obj * T_viz_offset.Inverse() * T_ee_abs.Inverse();

    /*std::cout << "T_abs_obj" << std::endl;
    std::cout << T_abs_obj.M.data[0] << "\t" <<	T_abs_obj.M.data[1] << "\t" <<	T_abs_obj.M.data[2] << "\t" << T_abs_obj.p.data[0] << std::endl;
    std::cout << T_abs_obj.M.data[3] << "\t" <<	T_abs_obj.M.data[4] << "\t" <<	T_abs_obj.M.data[5] << "\t" << T_abs_obj.p.data[1] << std::endl;
    std::cout << T_abs_obj.M.data[6] << "\t" <<	T_abs_obj.M.data[7] << "\t" <<	T_abs_obj.M.data[8] << "\t" << T_abs_obj.p.data[2] << std::endl;
    std::cout << std::endl;*/

    T_abs_obj.M.GetRPY(roll, pitch, yaw);

    std::cout << "abstract wrt object" << std::endl;
    std::cout << "\"xyz\": [" << T_abs_obj.p.data[0] << ",  " << T_abs_obj.p.data[1] << ",  " << T_abs_obj.p.data[2] << "]," << std::endl;
    std::cout << "\"rpy\": [" << roll << ",  " << pitch << ",  " << yaw << "]" << std::endl << std::endl;

    rate.sleep();

  }

  op_file_id.close();
  
  return 0;

}



