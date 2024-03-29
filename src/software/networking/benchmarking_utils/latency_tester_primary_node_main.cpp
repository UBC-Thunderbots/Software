#include <boost/program_options.hpp>
#include <iostream>

#include "software/networking/benchmarking_utils/latency_tester_primary_node.h"

int main(int argc, char **argv)
{
    struct CommandLineArgs
    {
        bool help = false;

        int listen_channel         = 0;
        unsigned short listen_port = 43000;
        int send_channel           = 1;
        unsigned short send_port   = 43001;
        int num_messages           = 100;
        int message_size_bytes     = 200;
        int timeout_duration_ms    = 10;
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
    desc.add_options()("num_messages",
                       boost::program_options::value<int>(&args.num_messages),
                       "The number of messages to send.");
    desc.add_options()("message_size_bytes",
                       boost::program_options::value<int>(&args.message_size_bytes),
                       "The size of the message to send in bytes.");
    desc.add_options()("timeout_duration_ms",
                       boost::program_options::value<int>(&args.timeout_duration_ms),
                       "The duration in milliseconds to wait for a response.");

    boost::program_options::variables_map vm;
    boost::program_options::store(parse_command_line(argc, argv, desc), vm);
    boost::program_options::notify(vm);

    if (args.help)
    {
        std::cout << desc << std::endl;
    }
    else
    {
        LatencyTesterPrimaryNode tester(
            args.listen_channel, args.listen_port, args.send_channel, args.send_port,
            args.message_size_bytes, std::chrono::milliseconds(args.timeout_duration_ms));
        tester.runTest(args.num_messages);
    }
}
