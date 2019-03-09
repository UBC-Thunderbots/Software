#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <thunderbots/aiConfig.h>
#include <thunderbots/xbox_controllerConfig.h>
#include <util/parameter/dynamic_parameter_utils.h>
#include <util/parameter/dynamic_parameters.h>

int main(int argc, char** argv)
{
    //ros::init(argc, argv, "dynamic_parameters");
    //ros::NodeHandle nh_a("/main_group/a/another");

    //dynamic_reconfigure::Server<param_server::SecondaryConfig> server2(nh_b);

    //// setup the subscriber to the updates topic to update parameters
    //ros::Subscriber param_update_subscription =
        //Util::DynamicParameters::initParamUpdateSubscription(nh_b);
    //ros::Subscriber param_update_subscription_2 =
        //Util::DynamicParameters::initParamUpdateSubscription(nh_a);

    //ros::spin();
    //return 0;
}
