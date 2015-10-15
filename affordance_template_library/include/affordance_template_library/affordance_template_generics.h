#ifndef _AT_GENERICS_H_
#define _AT_GENERICS_H_

namespace affordance_template_object
{
    struct Origin
    {
        double position[3]; // xyz
        double orientation[3]; // rpy
    };

    struct Shape
    {
        std::string color;
        double rgba[4];
        std::string type;
        std::string mesh; // if type == "mesh" then we need to fill the file name in here
        double size[3];
    };

    class Control
    {
    public:
        bool translation[3]; // xyz
        bool rotation[3]; // rpy
        double scale;

        inline std::string toBoolString(bool b) { return (b ? "true" : "false"); }
    };
}

#endif