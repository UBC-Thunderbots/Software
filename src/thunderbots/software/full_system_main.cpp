#include <ros/ros.h>

#include <boost/program_options.hpp>
#include <iostream>
#include <g3log/g3log.hpp>
#include <g3log/loglevels.hpp>

#include "ai/ai_wrapper.h"
#include "backend/backend_factory.h"
#include "backend/grsim_backend.h"
#include "backend/radio_backend.h"
#include "util/canvas_messenger/canvas_messenger.h"
#include "util/constants.h"
#include "util/logger/init.h"
#include "util/parameter/dynamic_parameter_utils.h"

using namespace boost::program_options;
// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    std::shared_ptr<AIWrapper> ai;
    std::shared_ptr<Backend> backend;

    std::shared_ptr<ros::NodeHandle> node_handle;
}  // namespace

void setBackendFromString(std::string backend_name)
{
    BackendFactory backend_factory;
    try {
        backend = backend_factory.createBackend(backend_name);
    } catch (const std::invalid_argument& e) {
        LOG(FATAL) << e.what();
    }
}

void parseCommandLineArgs(int argc, char **argv)
{
    std::vector<std::string> backend_names = BackendFactory().getRegisteredBackendNames();
    try
    {
        options_description desc{"Options"};
        desc.add_options()("help,h", "Help screen")
            // TODO: make backend a factory so we can get all the names here
            ("backend", value<std::string>()->notifier(setBackendFromString)->required(),
             "The backend that you would like to use");

        variables_map vm;
        store(parse_command_line(argc, argv, desc), vm);

        // We only process notifications if "help" was not given, which allows us to
        // avoid issues where required arguments are not required if "help" is given
        if (!vm.count("help"))
        {
            notify(vm);
        }
    }
    catch (const error &ex)
    {
        std::cerr << ex.what() << '\n';
    }
}

std::shared_ptr<ros::NodeHandle> initRos(int argc, char **argv)
{
    ros::init(argc, argv, "full_system");
    return std::make_shared<ros::NodeHandle>();
}

// TODO: javadoc comment here
void connectObservers()
{
    backend->Subject<World>::registerObserver(ai);
    ai->registerObserver(backend);
}

int main(int argc, char **argv)
{
    Util::Logger::LoggerSingleton::initializeLogger();

    node_handle = initRos(argc, argv);
    Util::CanvasMessenger::getInstance()->initializePublisher(*node_handle);

    ai = std::make_shared<AIWrapper>(*node_handle);

    parseCommandLineArgs(argc, argv);

    connectObservers();

    auto update_subscribers =
        Util::DynamicParameters::initUpdateSubscriptions(*node_handle);

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}
