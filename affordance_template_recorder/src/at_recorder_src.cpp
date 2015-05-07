#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
//#include <tf/tfScalar.h>
#include <iostream>
#include <fstream>
#include <kdl/frames.hpp>
#include <linux/input.h>
#include <fcntl.h>
#include <termios.h>
#include <vector>

std::string base_frame;
std::string target_frame;
std::string hand;

struct pose_struct{
	float x, y, z, roll, pitch, yaw;
};

std::vector<pose_struct> pose_vector;

int kbhit(void);
void record_pose();
void delete_pose();
void reset_array();
void save_to_jason();
bool process_user_option(char);

int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}

void record_pose()
{
	KDL::Frame T_ee_obj, T_abs_obj, T_abs_ee;
	KDL::Frame T_ee_abs;
	KDL::Frame T_viz_offset;

	tf::StampedTransform transform;
	geometry_msgs::Vector3 position;
	tf::Matrix3x3 tf_matrix;
	double roll, pitch, yaw;

	tf::TransformListener listener;

	pose_struct pose_struct_instance;

	try {
		listener.waitForTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform(base_frame, target_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
    	std::cout << "Error" << std::endl;
        ROS_ERROR("%s",ex.what());
       	ros::Duration(1.0).sleep();
       	return;
    }
    
    position.x = transform.getOrigin().x();
    position.y = transform.getOrigin().y();
    position.z = transform.getOrigin().z();

    tf_matrix.setRotation(transform.getRotation());
    tf_matrix.getRPY(roll, pitch, yaw);
    
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

    //T_ee_obj = T_abs_obj * T_ee_abs
    //T_abs_obj = T_ee_obj * inv(T_ee_abs)
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

    pose_struct_instance.x = T_abs_obj.p.data[0];
    pose_struct_instance.y = T_abs_obj.p.data[1];
    pose_struct_instance.z = T_abs_obj.p.data[2];
    pose_struct_instance.roll = roll;
    pose_struct_instance.pitch = pitch;
    pose_struct_instance.yaw = yaw;

    //pose_vector.resize(pose_vector.size() + 1);
    pose_vector.push_back(pose_struct_instance);

    std::cout << "Array size:" << pose_vector.size() << std::endl;

    save_to_jason();

}

void delete_pose()
{
	if (!pose_vector.empty())
		pose_vector.pop_back();
	else
		std::cout << "No poses stored in the scratch pad" << std::endl;

	save_to_jason();
}

void reset_array()
{
	if (!pose_vector.empty())
		pose_vector.clear();
	else
		std::cout << "No poses stored in the scratch pad" << std::endl;

	save_to_jason();
}

void save_to_jason()
{
	if (!pose_vector.empty()){
		std::cout << "Stored poses are:" << std::endl;
		for(int i=0; i<pose_vector.size(); i++)
			std::cout << "x:" << pose_vector[i].x << ",\t" << "y:" << pose_vector[i].y << ",\t" << "z:" << pose_vector[i].z << ",\t" 
				<< "roll:" << pose_vector[i].roll << ",\t" << "pitch:" << pose_vector[i].pitch << ",\t" << "yaw:" << pose_vector[i].yaw << "\n";
	}
	else
		std::cout << "No poses stored in the scratch pad" << std::endl;	
}

bool process_user_option(char user_input)
{
	switch(user_input)
	{
		case 'R':
		case 'r': record_pose();
				  break;

		case 'D':
		case 'd': delete_pose();
				  break;

		case 'T':
		case 't': reset_array();
				  break;

		case 'S':
		case 's': save_to_jason();
				  break;

		case 'Q':
    	case 'q': return false;

    	default: std::cout << std::endl;
    			 std::cout << "Please enter 'R', 'D', 'T', 'S', or 'Q' to quit" << std::endl;
	}

	return true;
}

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
  
  base_frame = argv[1];
  hand = argv[2];
  target_frame = "";

  if(hand=="right") {
    target_frame = "r_end";
  } else if(hand=="left") {
    target_frame = "l_end";
  } else {
    std::cout << "The third argument should be \"right\" or \"left\""  << std::endl;
    std::cout << "Quitting!" << std::endl;
    exit(1);	
  }
  
  bool menu_displayed=false;
  /*std::ofstream op_file_id;
  op_file_id.open("op_file.txt");*/
  
  ros::init(argc, argv, "record_traj_point_node");
  ros::NodeHandle node;
  ros::Rate rate(100);

  while (node.ok())
  {

    if (!menu_displayed)
    {
      std::cout << std::endl;
      std::cout << "AT keyboard interaction options:" << std::endl;
      std::cout << "Press 'R' or 'r' to record pose to scratch pad" << std::endl;
      std::cout << "Press 'D' or 'd' to delete previously recorded pose" << std::endl;
      std::cout << "Press 'T' or 't' to reset and start recording again" << std::endl;
      std::cout << "Press 'S' or 's' to save the recorded poses to the JASON file" << std::endl;
      std::cout << "Press 'Q' or 'q' to quit the program" << std::endl;

      menu_displayed=true;
    }

  	if (kbhit())
    {
    	std::cout << "key pressed" << std::endl;
      menu_displayed=false;
      char key_pressed = getchar();
      if(!process_user_option(key_pressed))
        break;
    }

    rate.sleep();

  }

  //op_file_id.close();
  
  return 0;

}



