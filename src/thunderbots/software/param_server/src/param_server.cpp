#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <thunderbots/ParamsConfig.h>
#include <util/parameter/dynamic_parameters.h>
#include <util/parameter/parameter.h>

// constants used by this node
namespace
{
    // specifies the rate at which the parameter values are refreshed
    constexpr int REFRESH_RATE_HZ = 1;
    // specifies the number of threads this node spins with
    constexpr int NUMBER_OF_THREADS = 1;
}  // namespace

/**
 * This callback is attatched to an update timer. All the changes are fetched
 * from the Parameter Server and the parameter objects are updated
 *
 */
void updateAllParameters(const ros::TimerEvent& event)
{
    Util::DynamicParameters::updateAllParametersFromROSParameterServer();
}

/**
 * This callback is attatched to the /parameter/parameter_updates topic
 * The new values arrive on this topic and the parameter objects are updated
 * from the Config msg
 *
 */
void parameterUpdateCallback(const dynamic_reconfigure::Config::ConstPtr& updates)
{
    Util::DynamicParameters::updateAllParametersFromConfigMsg(updates);
}

/**
 * This node starts up the dynamic_reconfigure server and properly configures
 * all the default values for all the parameters. Depending on the REFRESH_RATE
 * defined, updateROSParameters...() is called, fetching the values from the server
 *
 * For documentation on how to create new parameters and configure them to work
 * with the rqt_reconfigure node see this node's README.md
 *
 */
int main(int argc, char** argv)
{
    // init node
    ros::init(argc, argv, "dynamic_param");
    ros::NodeHandle node_handle;

    // setup dynamic reconfigure server
    dynamic_reconfigure::Server<param_server::ParamsConfig> server;

    ros::service::waitForService("/parameters/set_parameters", 1);
    ros::ServiceClient client =
        node_handle.serviceClient<dynamic_reconfigure::Reconfigure>(
            "/parameters/set_parameters");

    // create configuration message
    dynamic_reconfigure::Reconfigure srv;

    // get all configuration structs
    srv.request.config.ints    = Parameter<int32_t>::getConfigMsg().ints;
    srv.request.config.strs    = Parameter<std::string>::getConfigMsg().strs;
    srv.request.config.doubles = Parameter<double>::getConfigMsg().doubles;
    srv.request.config.bools   = Parameter<bool>::getConfigMsg().bools;

    // start the timer to update Parameters
    ros::Timer timer = node_handle.createTimer(
        ros::Duration(static_cast<double>(1 / REFRESH_RATE_HZ)), updateAllParameters);
    timer.start();

    // setup the subscriber to the updates topic to update parameters
    ros::Subscriber updater = node_handle.subscribe("/parameters/parameter_updates", 1,
                                                    parameterUpdateCallback);

    // spin asynchronously to allow for service call in the same node
    ros::AsyncSpinner spinner(NUMBER_OF_THREADS);
    spinner.start();

    // call the service to set params
    if (client.call(srv))
    {
        ROS_INFO("All parameters have been configured");
    }
    else
    {
        ROS_FATAL("Parameters have not been configured, showing default values");
        return 1;
    }

    ros::waitForShutdown();
    return 0;
}
