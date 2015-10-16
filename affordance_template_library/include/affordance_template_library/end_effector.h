#ifndef _AFFORDANCE_TEMPLATE_END_EFFECTOR_H_
#define _AFFORDANCE_TEMPLATE_END_EFFECTOR_H_

#include <affordance_template_library/affordance_template_generics.h>

namespace affordance_template_object
{
    struct Waypoint
    {
        int ee_pose;
        std::string display_object;
        Origin origin;
        Control controls;
    };

    struct EndEffector
    {
        int id;
        std::string name;
        std::vector<Waypoint> waypoints;
    };
}

#endif