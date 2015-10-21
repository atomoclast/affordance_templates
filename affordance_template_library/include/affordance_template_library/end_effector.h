#ifndef _AFFORDANCE_TEMPLATE_END_EFFECTOR_H_
#define _AFFORDANCE_TEMPLATE_END_EFFECTOR_H_

#include <affordance_template_library/affordance_template_generics.h>

namespace affordance_template_object
{
    struct EndEffectorWaypoint
    {
      int ee_pose;
      std::string display_object;
      Origin origin;
      Control controls;
    };

    struct EndEffectorWaypointList
    {
      int id;
      std::vector<EndEffectorWaypoint> waypoints;
    };

    struct Trajectory
    {
      std::string name;
      std::vector<affordance_template_object::EndEffectorWaypointList> ee_waypoint_list;
    };

}

#endif