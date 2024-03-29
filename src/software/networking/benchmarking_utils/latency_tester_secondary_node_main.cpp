#include "software/networking/benchmarking_utils/latency_tester_secondary_node.h"

#include <boost/program_options.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    struct CommandLineArgs
    {
        bool help = false;

        int listen_channel = 1; 
        unsigned short listen_port = 43001;
        int send_channel = 0;
        unsigned short send_port = 43000;
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("listen_channel",
                       boost::program_options::value<int>(&args.listen_channel),
                       "The channel to listen on.");
    desc.add_options()("listen_port",
            boost::program_options::value<unsigned short>(&args.listen_port),
            "The port to listen on.");
    desc.add_options()("send_channel",
            boost::program_options::value<int>(&args.send_channel),
            "The channel to send on.");
    desc.add_options()("send_port",
            boost::program_options::value<unsigned short>(&args.send_port),
            "The port to send on.");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }
    else
    {
        LatencyTesterSecondaryNode tester(args.listen_channel, args.listen_port, args.send_channel, args.send_port);
    }
}
