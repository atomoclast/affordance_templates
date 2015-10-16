#ifndef _DISPLAY_OBJECT_H_
#define _DISPLAY_OBJECT_H_

#include <affordance_template_library/affordance_template_generics.h>

namespace affordance_template_object
{
    struct DisplayObject
    {
        std::string name;
        std::string parent = "";
        Origin origin;
        Shape shape;
        Control controls;
    };
}

#endif