#pragma once

#include <ros/ros.h>

#include <memory>
#include <string>
#include <vector>

//messages for dynamic_reconfigure
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

//message that contains arrays of the xmlrpc types for reconf
#include <dynamic_reconfigure/Config.h>

//message for the reconfigure srv, takes a config msg
#include <dynamic_reconfigure/Reconfigure.h>


/**
 * This class defines a dynamic parameter, meaning the parameter
 * value can be changed during runtime. Although the class is templated, it is only meant
 * to support the types also supported by the ROS Parameter Server.
 *
 * See http://wiki.ros.org/Parameter%20Server for the list of types
 *
 * In our codebase, we support bool, int32_t, double, and strings
 * */

namespace{
        // what namespace the parameters will be in (ROS param server)
	std::string NamespaceForParameters = "/parameters";
}

template <class T>
class Parameter
{
    public:
        /**
         * Constructs a new Parameter
         *
         * @param parameter_name The name of the parameter used by dynamic_reconfigure
         * @param default_value The default value for this parameter
         */
        explicit Parameter<T>(const std::string& parameter_name, T default_value)
        {
            this->name_ = parameter_name;
            this->value_ = default_value;
	    this->internal_param_ = std::make_shared<Parameter<T>>(*this);

            Parameter<T>::registerParameter(this->internal_param_);
        }

        /**
         * Returns the global path in the ROS parameter server where this parameter is stored
         *
         * @return the global path in the ROS parameter sever where this parameter is stored
         */
        const std::string getROSParameterPath() const
        {
            return NamespaceForParameters + "/" + name();
        }

        /**
         * Returns the value of this parameter
         *
         * @return the value of this parameter
         */
        const T value() const
        {
	    if (internal_param_.get() != nullptr){
            	return this->internal_param_->value();
	    } else {
		return this->value_;
	    }
        }
        /**
         * Returns the name of this parameter
         *
         * @return the name of this parameter
         */
        const std::string name() const
        {
            return name_;
        }

        /**
         * Updates the value of this Parameter with the value from the ROS
         * Parameter Server
         */
        void updateValueFromROSParameterServer()
        {
            ros::param::get(getROSParameterPath(), value_);
        }

        /**
         * Sets the parameter value in the ROS Parameter Server to the new value.
         *
         * @param new_value The new value to be set
         */
        void setValueInROSParameterServer(T new_value)
        {
	    //dynamic reconfigure takes control of setting params 
	    //ros::param::set(getROSParameterPath(), new_value);
        }

        /**
         * Returns a reference to the Parameter registry. The registry is a list of
         * pointers to all the existing Parameters.
         *
         * @return An immutable reference to the Parameter registry
         */
        static const std::vector<std::shared_ptr<Parameter<T>>>& getRegistry()
        {
            return Parameter<T>::getMutableRegistry();
        }

        /**
         * Returns a reference to the config struct. The config struct contains
         * all the current configurations
         *
         * @return An immutable reference to the Config struct
         */
        static const dynamic_reconfigure::Config& getConfigStruct()
        {
            return Parameter<T>::getMutableConfigStruct();
        }

        /**
         * Registers (adds) a Parameter to the registry. Since the shared pointer is moved
         * into the registry, the pointer may not be accessed by the caller after this
         * function has been called.
	 *
	 * Also registers params to the static configuration struct used to 
	 * set parameters
         *
         * @param parameter A shared pointer to the Parameter to add. This pointer may not
         * be accessed by the caller after this function has been called.
         */
        static void registerParameter(std::shared_ptr<Parameter<T>> parameter)
        {   

            if constexpr(std::is_same<T, bool>::value){   
                dynamic_reconfigure::BoolParameter bool_msg;

                bool_msg.name = parameter->name();
                bool_msg.value = static_cast<bool>(parameter->value());

                Parameter<T>::getMutableConfigStruct().bools.push_back(bool_msg);
            }

	    else if  constexpr(std::is_same<T, int32_t>::value){   
                dynamic_reconfigure::IntParameter int_msg;

                int_msg.name = parameter->name();
                int_msg.value = static_cast<int32_t>(parameter->value());

                Parameter<T>::getMutableConfigStruct().ints.push_back(int_msg);
            }

	    else if constexpr(std::is_same<T, double>::value){   
                dynamic_reconfigure::DoubleParameter double_msg;

                double_msg.name = parameter->name();
                double_msg.value = static_cast<double>(parameter->value());

                Parameter<T>::getMutableConfigStruct().doubles.push_back(double_msg);
            }

	    else if constexpr(std::is_same<T, std::string>::value){   
                dynamic_reconfigure::StrParameter str_msg;

                str_msg.name = parameter->name();
                str_msg.value = static_cast<std::string>(parameter->value());

                Parameter<T>::getMutableConfigStruct().strs.push_back(str_msg);
            }

	    else {
		ROS_WARN("attempting to configure with unkown type");
	    }

            Parameter<T>::getMutableRegistry().emplace_back(parameter);
        }

        /**
         * Updates all the Parameters of type T with the latest values from the ROS
         * Parameter Server
         */
        static void updateAllParametersFromROSParameterServer()
        {
            for (const auto& p : Parameter<T>::getRegistry())
            {
                p->updateValueFromROSParameterServer();
            }
        }

    private:
        /**
         * Returns a mutable reference to the Parameter registry. This is the same as the
         * above getRegistry() function, except that it returns a mutable reference. We need a
         * mutable version so that member functions can modify the registry. The function is
         * private so that external sources cannot modify the registry in unexpected ways.
         *
         * @return A mutable reference to the Parameter registry
         */
        static std::vector<std::shared_ptr<Parameter>>& getMutableRegistry()
        {
            static std::vector<std::shared_ptr<Parameter<T>>> instance;
            return instance;
        }

        // Store the value so it can be retrieved without fetching from the server again
        T value_;
        std::string name_;
	std::shared_ptr<Parameter<T>> internal_param_;

        /**
         * Returns a mutable configuration struct that will hold all the 
         * information related to the parameters created
         * Struct contains bool,strs,ints,doubles vectors which are inherently mutable
         *
         * @return A mutable reference to the configuration struct
         */
        static dynamic_reconfigure::Config& getMutableConfigStruct()
        {
            static dynamic_reconfigure::Config config;
            return config;
        }

};
