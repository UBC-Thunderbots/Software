#include "util/parameter/integer/integer_parameter.h"
#include <ros/ros.h>

IntegerParameter::IntegerParameter(const std::string& ros_parameter_path,
                                   int default_value)
    : value_(default_value)
{
    // Because ros_parameter_path is defined in the Parameter superclass, it can't go in
    // the initializer list
    this->ros_parameter_path = ros_parameter_path;

    IntegerParameter::registerParameter(std::shared_ptr<IntegerParameter>(this));
}

const int IntegerParameter::value() const
{
    return value_;
}

void IntegerParameter::updateValueFromParameterServer()
{
    ros::param::get(getROSParameterPath(), value_);
}

void IntegerParameter::setValueInParameterServer(int new_value) const
{
    ros::param::set(getROSParameterPath(), new_value);
}

const std::vector<std::shared_ptr<IntegerParameter>>& IntegerParameter::getRegistry()
{
    return IntegerParameter::getMutableRegistry();
}

std::vector<std::shared_ptr<IntegerParameter>>& IntegerParameter::getMutableRegistry()
{
    static std::vector<std::shared_ptr<IntegerParameter>> instance;
    return instance;
}

void IntegerParameter::registerParameter(std::shared_ptr<IntegerParameter> integer_param)
{
    IntegerParameter::getMutableRegistry().emplace_back(integer_param);
}
