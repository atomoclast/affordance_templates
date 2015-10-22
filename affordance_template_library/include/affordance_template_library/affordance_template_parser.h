#ifndef _AT_PARSER_H_
#define _AT_PARSER_H_

#include <ros/ros.h>

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include <rapidjson/document.h>
#include <rapidjson/filereadstream.h>

#include <affordance_template_library/affordance_template_structure.h>

using namespace rapidjson;

namespace affordance_template_object
{
    class AffordanceTemplateParser
    {

    public: 
			AffordanceTemplateParser();
			~AffordanceTemplateParser();

			bool loadFromFile(std::string filename, AffordanceTemplateStructure &at);

    };
}

#endif