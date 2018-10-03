#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <util/parameter/parameter.h>
#include <thunderbots/ParamsConfig.h>

int main(int argc, char **argv) {
    //init node
    ros::init(argc, argv, "dynamic_param");
    ros::NodeHandle node_handle;

    
    //setup dynamic reconfigure server
    dynamic_reconfigure::Server<param_server::ParamsConfig> server;
    ros::service::waitForService("/param_server/set_parameters", 1);
    ros::ServiceClient client = node_handle.serviceClient<dynamic_reconfigure::Reconfigure>("/param_server/set_parameters");

    //create configuration message
    dynamic_reconfigure::Reconfigure srv;

    //get all configuration structs
    srv.request.config.ints = Parameter<int>::getConfigStruct().ints;
    srv.request.config.strs = Parameter<std::string>::getConfigStruct().strs;
    srv.request.config.doubles = Parameter<double>::getConfigStruct().doubles;
    srv.request.config.bools = Parameter<bool>::getConfigStruct().bools;

    //call the service to set params
    if(client.call(srv)){
        ROS_INFO("all parameters have been configured");
    } else {
        ROS_FATAL("paramtrs have not been configured, showing cached values");
    }

    ROS_INFO("Spinning node");
    ros::spin();
    return 0;
}
