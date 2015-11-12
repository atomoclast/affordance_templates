#include <affordance_template_markers/affordance_template_planning.h>

using namespace affordance_template_action;

PlanningServer::PlanningServer(const std::string& server_name) :
    action_server_(nh_, server_name, boost::bind(&PlanningServer::planRequest, this, _1), false)
{
    ROS_INFO("[PlanningServer] creating planning action server: %s. no robot interface was provided; creating one now.", server_name.c_str());

    std::string robot_yaml = "";
    nh_.getParam("robot_config", robot_yaml);

    if (robot_yaml.empty())
    {
        ROS_WARN("[PlanningServer] no robot_config found on param server. setting to default -- r2_upperbody.yaml");
        robot_yaml = "r2_upperbody.yaml";
    }

    std::string root = ros::package::getPath("affordance_template_library");
    if (root.empty())
    {
        ROS_FATAL("[PlanningServer] error finding package path: affordance_template_library");
        return;
    }

    root += "/robots/";

    if (!boost::filesystem::exists(root) || !boost::filesystem::is_directory(root))
    {
        ROS_WARN("[PlanningServer] cannot find any robots in path: %s!!", root.c_str());
        return;
    }

    // make robot instances with the .yamls we just found
    robot_interface_.reset(new affordance_template_markers::RobotInterface());
    if (!robot_interface_->load(root + robot_yaml))
    {
        ROS_ERROR("[PlanningServer] robot yaml %s NOT loaded, ignoring.", robot_yaml.c_str());
        return;
    }

    if ( !robot_interface_->configure())
    {
        ROS_WARN("[PlanningServer] robot yaml %s NOT configured, ignoring.", robot_yaml.c_str());
        return;
    } 

    action_server_.start();
}

PlanningServer::PlanningServer(const std::string& server_name, boost::shared_ptr<affordance_template_markers::RobotInterface> _robot_interface) :
    action_server_(nh_, server_name, boost::bind(&PlanningServer::planRequest, this, _1), false),
    robot_interface_(_robot_interface)
{
    ROS_INFO("[PlanningServer] creating planning action server: %s", server_name.c_str());
    action_server_.start();
}

void PlanningServer::planRequest(const affordance_template_msgs::PlanGoalConstPtr& goal)
{
    // affordance_template_msgs::PlanResult
    // action_server_.setSucceeded();
    // action_server_.setAborted();
    // affordance_template_msgs::PlanFeedback
    // action_server_.publishFeedback();
    // action_server_.setPreempted();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_at_action_server");

    PlanningServer as("affordance_template_planning_server");
    ros::spin();

    return 0;
}
