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
    std::cout << "When running exe, the format should be \"exe <base-frame> <target-frame>\"" << std::endl;
    std::cout << "e.g. ./at_recorder Valve:0 r_default_tool" << std::endl;
    std::cout << "Quitting!" << std::endl;

    exit(1);
  }
  
  std::string base_frame = argv[1];
  std::string target_frame = argv[2];

  std::ofstream op_file_id;
  op_file_id.open("op_file.txt");

  KDL::Frame T_ee_obj, T_abs_obj, T_abs_ee;
  KDL::Frame T_ee_abs;
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

    tf_matrix.setRotation(transform.getRotation());
    tf_matrix.getRPY(roll, pitch, yaw);
    tf_matrix.getEulerYPR(yaw, pitch, roll);

    //Populate the translation elements of T_abs_obj

    T_ee_obj.p.data[0] = position.x;
    T_ee_obj.p.data[1] = position.y;
    T_ee_obj.p.data[2] = position.z;

    //Populate the rotation elements of T_abs_obj

    /*tf::Vector3 tf_matrix_row[3];
    tf_matrix_row[0] = tf_matrix.getRow(0);
    tf_matrix_row[1] = tf_matrix.getRow(1);
    tf_matrix_row[2] = tf_matrix.getRow(2);
 
    T_ee_obj.M.data[0] = tf_matrix_row[0].getX();	T_ee_obj.M.data[1] = tf_matrix_row[0].getY();	T_ee_obj.M.data[2] = tf_matrix_row[0].getZ();
    T_ee_obj.M.data[3] = tf_matrix_row[1].getX();	T_ee_obj.M.data[4] = tf_matrix_row[1].getY();	T_ee_obj.M.data[5] = tf_matrix_row[1].getZ();
    T_ee_obj.M.data[6] = tf_matrix_row[2].getX();	T_ee_obj.M.data[7] = tf_matrix_row[2].getY();	T_ee_obj.M.data[8] = tf_matrix_row[2].getZ();*/

    T_ee_obj.M.RPY(roll, pitch, yaw);

    
    std::cout << position.x << "\t" << position.y << "\t" << position.z << "\t" << std::endl;
    /*std::cout << roll << "\t" << pitch << "\t" << yaw << "\t" << std::endl;
    
    op_file_id << position.x << "\t" << position.y << "\t" << position.z << "\n";
    op_file_id << roll << "\t" << pitch << "\t" << yaw << "\n\n";*/

    //Populate the translation elements of T_abs_ee

    T_ee_abs.p.data[0] = 0.1;
    T_ee_abs.p.data[1] = -0.2;
    T_ee_abs.p.data[2] = 0.0;

    //Populate the rotation elements of T_abs_ee 

    /*tf_matrix.setRPY(0.0, 1.57, 3.14);

    tf_matrix_row[0] = tf_matrix.getRow(0);
    tf_matrix_row[1] = tf_matrix.getRow(1);
    tf_matrix_row[2] = tf_matrix.getRow(2);

    T_ee_abs.M.data[0] = tf_matrix_row[0].getX();	T_ee_abs.M.data[1] = tf_matrix_row[0].getY();	T_ee_abs.M.data[2] = tf_matrix_row[0].getZ();
    T_ee_abs.M.data[3] = tf_matrix_row[1].getX();	T_ee_abs.M.data[4] = tf_matrix_row[1].getY();	T_ee_abs.M.data[5] = tf_matrix_row[1].getZ();
    T_ee_abs.M.data[6] = tf_matrix_row[2].getX();	T_ee_abs.M.data[7] = tf_matrix_row[2].getY();	T_ee_abs.M.data[8] = tf_matrix_row[2].getZ();*/

    T_ee_abs.M.RPY(0.0, 1.57, 3.14);

    T_abs_obj = T_ee_obj *  T_ee_abs.Inverse();

    std::cout << T_ee_obj.M.data[0] << "\t" <<	T_ee_obj.M.data[1] << "\t" <<	T_ee_obj.M.data[2] << "\t" << T_ee_obj.p.data[0] << std::endl;
    std::cout << T_ee_obj.M.data[3] << "\t" <<	T_ee_obj.M.data[4] << "\t" <<	T_ee_obj.M.data[5] << "\t" << T_ee_obj.p.data[1] << std::endl;
    std::cout << T_ee_obj.M.data[6] << "\t" <<	T_ee_obj.M.data[7] << "\t" <<	T_ee_obj.M.data[8] << "\t" << T_ee_obj.p.data[2] << std::endl;
    std::cout << std::endl;

    rate.sleep();

  }

  op_file_id.close();
  
  return 0;

}



