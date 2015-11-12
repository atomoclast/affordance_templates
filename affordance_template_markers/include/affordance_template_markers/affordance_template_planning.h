#ifndef _AFFORDANCE_TEMPLATE_ACTION_H_
#define _AFFORDANCE_TEMPLATE_ACTION_H_

#include <ros/ros.h>
#include <ros/package.h>

#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>

#include <actionlib/server/simple_action_server.h>
#include <affordance_template_msgs/PlanAction.h>
#include <affordance_template_markers/robot_interface.h>

namespace affordance_template_action
{
    class PlanningServer
    {
        void planRequest(const affordance_template_msgs::PlanGoalConstPtr&);

        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<affordance_template_msgs::PlanAction> action_server_;
        boost::shared_ptr<affordance_template_markers::RobotInterface> robot_interface_;

    public:
        PlanningServer(const std::string&);
        PlanningServer(const std::string&, boost::shared_ptr<affordance_template_markers::RobotInterface>);
        ~PlanningServer(){}
    };
}

#endif