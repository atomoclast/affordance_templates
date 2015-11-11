#ifndef _AT_PARSER_H_
#define _AT_PARSER_H_

#include <ros/ros.h>

#include <iostream>
#include <fstream>

#include <boost/thread.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>
#include <rapidjson/filereadstream.h>

#include <affordance_template_library/affordance_template_structure.h>
#include <affordance_template_library/affordance_template_generics.h>

using namespace rapidjson;

namespace affordance_template_object
{
    class AffordanceTemplateParser
    {
    public: 
        AffordanceTemplateParser(){}
        ~AffordanceTemplateParser(){}

        bool loadFromFile(const std::string&, AffordanceTemplateStructure &at);
        bool saveToFile(const std::string&, const AffordanceTemplateStructure&);
    };
}

#endif