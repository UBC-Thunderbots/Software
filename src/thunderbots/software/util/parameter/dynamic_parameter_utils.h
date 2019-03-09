#include <dynamic_reconfigure/Reconfigure.h>
#include <ros/ros.h>

#include "util/parameter/parameter.h"

namespace Util::DynamicParameters
{
    void init();
    void initReconfigureServers();
    extern std::vector<std::string> cfg_strs;

    /**
     * This callback is attatched to the /parameter/parameter_updates topic
     * The new values arrive on this topic and the parameter objects are updated
     * from the Config msg
     */
    void parameterUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updates);

    /**
     * Updates all known parameters with the latest values from the ROS Parameter
     * Server
     */
    void updateAllParametersFromROSParameterServer();

    /**
     * Updates all known parameters with the latest values from config lists
     * in the dynamic_reconfigure::Config msgs
     *
     */
    void updateAllParametersFromConfigMsg(const dynamic_reconfigure::Config::ConstPtr&);
}  // namespace Util::DynamicParameters
