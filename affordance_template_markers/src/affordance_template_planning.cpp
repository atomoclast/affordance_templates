#include <affordance_template_markers/affordance_template_planning.h>

using namespace affordance_template_action;

PlanningServer::PlanningServer(const std::string& server_name) :
    action_server_(nh_, server_name, boost::bind(&PlanningServer::planRequest, this, _1), false)
{    
    action_server_.start();   
}

void PlanningServer::planRequest(const affordance_template_msgs::PlanGoalConstPtr& goal)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_at_action_server");

    PlanningServer as("affordance_template_planning_server");
    ros::spin();

    return 0;
}
