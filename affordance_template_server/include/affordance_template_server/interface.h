#ifndef _AFFORDANCE_TEMPLATE_INTERFACE_H_
#define _AFFORDANCE_TEMPLATE_INTERFACE_H_

#include <affordance_template_server/server.h>

namespace affordance_template_server
{
    class AffordanceTemplateInterface
    {
        AffordanceTemplateServer server_;

    public:
        AffordanceTemplateInterface();
        ~AffordanceTemplateInterface();
    };
}

#endif