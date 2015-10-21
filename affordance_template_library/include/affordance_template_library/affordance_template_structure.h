#ifndef _AFFORDANCE_TEMPLATE_STRUCTURE_H_
#define _AFFORDANCE_TEMPLATE_STRUCTURE_H_

#include <affordance_template_library/display_object.h>
#include <affordance_template_library/end_effector.h>
#include <affordance_template_library/affordance_template_generics.h>

namespace affordance_template_object
{
	typedef std::vector<affordance_template_object::Trajectory> TrajectoryList;
	typedef std::vector<affordance_template_object::DisplayObject> DisplayObjectList;

    struct AffordanceTemplateStructure
    {
        std::string name;
        std::string image;
        std::string filename;
        TrajectoryList ee_trajectories;
      	DisplayObjectList display_objects;
          
    };
}

#endif