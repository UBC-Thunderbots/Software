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
        int listen_channel         = 1;
        unsigned short listen_port = 43001;
        int send_channel           = 0;
        unsigned short send_port   = 43000;
        std::string send_ip        = "";
        bool multicast             = false;
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("interface",
                       boost::program_options::value<std::string>(&args.interface),
                       "The interface to bind to.");
    desc.add_options()("listen_port",
                       boost::program_options::value<unsigned short>(&args.listen_port),
                       "The port to listen on.");
    desc.add_options()("send_port",
                       boost::program_options::value<unsigned short>(&args.send_port),
                       "The port to send on.");

    desc.add_options()("multicast",
            boost::program_options::value<bool>(&args.multicast),
            "Use multicast instead of unicast for communication.");

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
        std::unique_ptr<LatencyTesterSecondaryNode> tester;
        if (args.multicast)
        {
            tester = std::make_unique<LatencyTesterSecondaryNode>(args.interface, args.listen_channel,
                                                                  args.listen_port, args.send_channel, args.send_port);
        }
        else
        {
            tester = std::make_unique<LatencyTesterSecondaryNode>(args.interface, args.listen_port, args.send_ip,
                    args.send_port);
        }

        while (true)
        {
            std::this_thread::sleep_for(std::chrono::seconds(SLEEP_DURATION_S));
        }
    }
}
