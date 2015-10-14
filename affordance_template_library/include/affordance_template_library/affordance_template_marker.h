#ifndef _AT_MARKER_H_
#define _AT_MARKER_H_

#include <affordance_template_library/affordance_template_generics.h>

namespace affordance_template_object
{
    struct AffordanceTemplateMarker
    {
        std::string name;
        Origin origin;
        Shape shape;
        Control controls;
    };
}

#endif