#include "util/parameter/boolean/boolean_parameter.h"
#include <ros/ros.h>

BooleanParameter::BooleanParameter(const std::string& ros_parameter_path,
                                   bool default_value)
    : value_(default_value)
{
    // Because ros_parameter_path is defined in the Parameter superclass, it can't go in
    // the initializer list
    this->ros_parameter_path = ros_parameter_path;

    BooleanParameter::registerParameter(std::shared_ptr<BooleanParameter>(this));
}

const bool BooleanParameter::value() const
{
    return value_;
}

void BooleanParameter::updateValueFromParameterServer()
{
    ros::param::get(getROSParameterPath(), value_);
}

void BooleanParameter::setValueInParameterServer(bool new_value) const
{
    ros::param::set(getROSParameterPath(), new_value);
}

const std::vector<std::shared_ptr<BooleanParameter>>& BooleanParameter::getRegistry()
{
    return BooleanParameter::getMutableRegistry();
}

std::vector<std::shared_ptr<BooleanParameter>>& BooleanParameter::getMutableRegistry()
{
    static std::vector<std::shared_ptr<BooleanParameter>> instance;
    return instance;
}

void BooleanParameter::registerParameter(std::shared_ptr<BooleanParameter> bool_param)
{
    BooleanParameter::getMutableRegistry().emplace_back(bool_param);
}
