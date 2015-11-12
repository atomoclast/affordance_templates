#include <affordance_template_markers/affordance_template_action.h>

using namespace affordance_template_action;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_at_action_server");

    ActionServer as();
    // ros::spin();

    return 0;
}
