#ifndef _AFFORDANCE_TEMPLATE_ACTION_H_
#define _AFFORDANCE_TEMPLATE_ACTION_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <affordance_template_msgs/PlanAction.h>

namespace affordance_template_action
{
    class ActionServer
    {
    public:
        ActionServer(){}
        ~ActionServer(){}
    };
}

#endif