#include "software/logger/logger.h"
#include "software/networking/benchmarking_utils/latency_tester_secondary_node.h"

#include <boost/program_options.hpp>
#include <iostream>

constexpr int SLEEP_DURATION_S = 1;

int main(int argc, char **argv)
{
    struct CommandLineArgs
    {
        bool help = false;

        std::string interface = "";
        std::string runtime_dir = "/tmp/tbots";
        int listen_channel         = 1;
        unsigned short listen_port = 43001;
        int send_channel           = 0;
        unsigned short send_port   = 43000;
        std::string send_ip        = "";
        bool unicast             = false;
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("runtime_dir", boost::program_options::value<std::string>(&args.runtime_dir),
                       "The directory to output logs.");
    desc.add_options()("interface",
                       boost::program_options::value<std::string>(&args.interface),
                       "The interface to bind to.");
    desc.add_options()("listen_port",
                       boost::program_options::value<unsigned short>(&args.listen_port),
                       "The port to listen on.");
    desc.add_options()("send_port",
                       boost::program_options::value<unsigned short>(&args.send_port),
                       "The port to send on.");

    desc.add_options()("unicast",
            boost::program_options::value<bool>(&args.unicast),
            "Use unicast instead of multicast for communication.");

    // Multicast options
    desc.add_options()("listen_channel",
                       boost::program_options::value<int>(&args.listen_channel),
                       "The channel to listen on.");
    desc.add_options()("send_channel",
                       boost::program_options::value<int>(&args.send_channel),
                       "The channel to send on.");

    // Unicast options
    desc.add_options()("send_ip",
            boost::program_options::value<std::string>(&args.send_ip),
            "The IP to send the reflected packets to.");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }
    else
    {
        LoggerSingleton::initializeLogger(args.runtime_dir, nullptr, false);

        std::unique_ptr<LatencyTesterSecondaryNode> tester;
        if (args.unicast)
        {
            tester = std::make_unique<LatencyTesterSecondaryNode>(args.interface, args.listen_port, args.send_ip,
                    args.send_port);
        }
        else
        {
            LOG(INFO) << "Creating multicast latency tester";
            LOG(INFO) << "Listen channel: " << args.listen_channel << ", Listen port: " << args.listen_port;
            LOG(INFO) << "Send channel: " << args.send_channel << ", Send port: " << args.send_port;

            tester = std::make_unique<LatencyTesterSecondaryNode>(args.interface, args.listen_channel,
                                                                  args.listen_port, args.send_channel, args.send_port);
        }

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(SLEEP_DURATION_S));
        }
    }
}
