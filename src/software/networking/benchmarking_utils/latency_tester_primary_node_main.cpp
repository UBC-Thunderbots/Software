#include "software/logger/logger.h"
#include "software/networking/benchmarking_utils/latency_tester_primary_node.h"

#include <boost/program_options.hpp>
#include <iostream>


int main(int argc, char **argv)
{
    struct CommandLineArgs
    {
        bool help = false;

        std::string interface = "";
        std::string runtime_dir = "/tmp/tbots";
        int listen_channel         = 0;
        unsigned short listen_port = 43000;
        int send_channel           = 1;
        unsigned short send_port   = 43001;
        int num_messages           = 100;
        int message_size_bytes     = 200;
        int timeout_duration_ms    = 10;
        int initial_delay_s        = 20;
        bool unicast             = false;
        std::string send_ip        = "";
    };

    CommandLineArgs args;
    boost::program_options::options_description desc{"Options"};

    desc.add_options()("help,h", boost::program_options::bool_switch(&args.help),
                       "Help screen");
    desc.add_options()("runtime_dir",
                       boost::program_options::value<std::string>(&args.runtime_dir),
                       "The directory to output logs.");
    desc.add_options()("listen_port",
                       boost::program_options::value<unsigned short>(&args.listen_port),
                       "The port to listen on.");
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
    desc.add_options()("initial_delay_s",
            boost::program_options::value<int>(&args.initial_delay_s),
            "The initial delay to wait before sending packets (s)");
    desc.add_options()("unicast",
            boost::program_options::value<bool>(&args.unicast),
            "Use unicast instead of multicast for communication.");
    desc.add_options()("interface",
                       boost::program_options::value<std::string>(&args.interface),
                       "The interface to bind to.");

    // Multicast Options
    desc.add_options()("listen_channel",
                       boost::program_options::value<int>(&args.listen_channel),
                       "The channel to listen on.");
    desc.add_options()("send_channel",
                       boost::program_options::value<int>(&args.send_channel),
                       "The channel to send on.");

    // Unicast Options
    desc.add_options()("send_ip",
                       boost::program_options::value<std::string>(&args.send_ip),
                       "The IP of the receiver for unicast communication.");

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

        std::chrono::milliseconds timeout_duration_ms(args.timeout_duration_ms);
        std::unique_ptr<LatencyTesterPrimaryNode> tester;
        if (args.unicast)
        {
            tester = std::make_unique<LatencyTesterPrimaryNode>(args.interface, args.listen_port, args.send_ip,
                    args.send_port, args.message_size_bytes, timeout_duration_ms);
        }
        else
        {
            LOG(INFO) << "Running multicast latency test";
            LOG(INFO) << "Listen channel: " << args.listen_channel << ", Listen port: " << args.listen_port;
            LOG(INFO) << "Send channel: " << args.send_channel << ", Send port: " << args.send_port;

            tester = std::make_unique<LatencyTesterPrimaryNode>(args.interface,
                args.listen_channel, args.listen_port, args.send_channel, args.send_port,
                args.message_size_bytes, timeout_duration_ms);
        }

        std::this_thread::sleep_for(std::chrono::seconds(args.initial_delay_s));
        tester->runTest(args.num_messages);
    }
}
