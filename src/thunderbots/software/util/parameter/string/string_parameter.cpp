#include "util/parameter/string/string_parameter.h"
#include <ros/ros.h>

StringParameter::StringParameter(const std::string& ros_parameter_path,
                                 std::string default_value)
    : value_(default_value)
{
    // Because ros_parameter_path is defined in the Parameter superclass, it can't go in
    // the initializer list
    this->ros_parameter_path = ros_parameter_path;

    StringParameter::registerParameter(std::shared_ptr<StringParameter>(this));
}

const std::string StringParameter::value() const
{
    return value_;
}

void StringParameter::updateValueFromParameterServer()
{
    ros::param::get(getROSParameterPath(), value_);
}

void StringParameter::setValueInParameterServer(std::string new_value) const
{
    ros::param::set(getROSParameterPath(), new_value);
}

const std::vector<std::shared_ptr<StringParameter>>& StringParameter::getRegistry()
{
    return StringParameter::getMutableRegistry();
}

std::vector<std::shared_ptr<StringParameter>>& StringParameter::getMutableRegistry()
{
    static std::vector<std::shared_ptr<StringParameter>> instance;
    return instance;
}

void StringParameter::registerParameter(std::shared_ptr<StringParameter> string_param)
{
    StringParameter::getMutableRegistry().emplace_back(string_param);
}
