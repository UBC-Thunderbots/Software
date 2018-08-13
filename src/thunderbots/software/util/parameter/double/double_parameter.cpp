#include "util/parameter/double/double_parameter.h"
#include <ros/ros.h>

DoubleParameter::DoubleParameter(const std::string& ros_parameter_path,
                                 double default_value)
    : value_(default_value)
{
    // Because ros_parameter_path is defined in the Parameter superclass, it can't go in
    // the initializer list
    this->ros_parameter_path = ros_parameter_path;

    DoubleParameter::registerParameter(std::shared_ptr<DoubleParameter>(this));
}

const double DoubleParameter::value() const
{
    return value_;
}

void DoubleParameter::updateValueFromParameterServer()
{
    ros::param::get(getROSParameterPath(), value_);
}

void DoubleParameter::setValueInParameterServer(double new_value) const
{
    ros::param::set(getROSParameterPath(), new_value);
}

const std::vector<std::shared_ptr<DoubleParameter>>& DoubleParameter::getRegistry()
{
    return DoubleParameter::getMutableRegistry();
}

std::vector<std::shared_ptr<DoubleParameter>>& DoubleParameter::getMutableRegistry()
{
    static std::vector<std::shared_ptr<DoubleParameter>> instance;
    return instance;
}

void DoubleParameter::registerParameter(std::shared_ptr<DoubleParameter> double_param)
{
    DoubleParameter::getMutableRegistry().emplace_back(double_param);
}
