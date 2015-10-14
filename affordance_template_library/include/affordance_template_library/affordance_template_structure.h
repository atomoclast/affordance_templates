#ifndef _AT_STRUCTURE_H_
#define _AT_STRUCTURE_H_

#include <affordance_template_library/affordance_template_marker.h>
#include <affordance_template_library/end_effector.h>
#include <affordance_template_library/affordance_template_generics.h>

namespace affordance_template_object
{
    struct AffordanceTemplateStructure
    {
        std::string name;
        std::string image;
        std::string filename;
        std::vector<affordance_template_object::EndEffector> ee_trajectories;
        std::vector<affordance_template_object::AffordanceTemplateMarker> display_objects;
    };
}

#endif