# WiFi Communication README

## Lessons Learned So Far
1. For optimal performance, make sure that any packets sent over the network are below MTU size (1500 bytes). Packets larger than MTU size require multiple transmissions for a single send event and multiple retransmissions in the case of packet loss. Overall, these packets contribute to greater utilization of the network and increased latency.
2. Connect the host computer to the network via ethernet cable when possible. By minimizing the utilization of the WiFi network, this change significantly improves the round-trip time.
3. Use unicast communication over WiFi for frequent, low-latency communication over multicast. [RFC 9119](https://www.rfc-editor.org/rfc/rfc9119.html#section-3.1.2) provides a good overview of the limitations of multicast communication over WiFi. In short, routers are forced to transmit at the lowest common data rate of all devices on the network to ensure that all devices receive the packet, meaning that the network is slowed down by the slowest device. In addition, router features such as Multiple Input Multiple Output (MIMO) may not be available when using multicast communication. We have found a 24% improvement in round-trip time when switching from multicast to unicast communication with some benchmarking tests.
4. On embedded Linux devices, WiFi power management seems to cause significant latency spikes. To disable power management, run the following command: `sudo iw dev {wifi_interface} set power_save off` where `{wifi_interface}` is the name of the WiFi interface (e.g. `wlan0`).


## Debugging

We have built some tools to help diagnose network latency problems without the confounding effects of running Thunderloop and Thunderscope and their associated overheads.

The latency tester tests the round trip time between two nodes. The primary node sends a message to the secondary node, which sends back the same message as soon as it receives it. The primary node then measures the round trip time.

Typically, the primary node is the host computer and the secondary node is the robot.

## Running the latency tester with the robot
### Prerequisites
You must know:
- The IP address of the robot. We will refer to this address as `{robot_ip}`.
- The WiFi interface of the robot. We will refer to this interface as `{robot_wifi_interface}`. This interface is typically found by running `ifconfig` or `ip a` on the robot.
- The network interface of the host computer. We will refer to this interface as `{host_interface}`. This interface is typically found by running `ifconfig` or `ip a` on the host computer.

1. Build the latency tester secondary node: `./tbots.py build latency_tester_secondary_node --platforms=//cc_toolchain:robot`
2. Copy the binary to the robot: `scp bazel-bin/software/networking/benchmarking_utils/latency_tester_secondary_node robot@{robot_ip}:/home/robot/latency_tester_secondary_node`
3. SSH into the robot: `ssh robot@{robot_ip}`
4. There are two test modes: multicast or unicast
    1. For multicast:
        1. Run the latency tester secondary node: `./latency_tester_secondary_node --interface {robot_wifi_interface}`
            - You may optionally also provide the following arguments:
                - `--runtime_dir` to specify the directory where log files are stored
                - `--listen_port` to specify the port on which the secondary node listens for messages.
                - `--send_port` to specify the port on which the secondary node sends messages
                - `--listen_channel` to specify the channel on which the secondary node listens for messages
                - `--send_channel` to specify the channel on which the secondary node sends back replies
        2. On a different terminal on the host computer, run the latency tester primary node: `./tbots.py run latency_tester_primary_node -- --interface {host_interface}`
            - You may optionally also provide the following arguments:
                - `--runtime_dir` to specify the directory where log files are stored
                - `--listen_port` to specify the port on which the primary node listens for replies to messages. This port must match the `--send_port` argument provided to the secondary node.
                - `--send_port` to specify the port on which the primary node sends messages. This port must match the `--listen_port` argument provided to the secondary node.
                - `--listen_channel` to specify the channel on which the primary node listens for replies to messages. This channel must match the `--send_channel` argument provided to the secondary node.
                - `--send_channel` to specify the channel on which the primary node sends messages. This channel must match the `--listen_channel` argument provided to the secondary node.
                - `--num_messages` to specify the number of messages to send
                - `--message_size_bytes` to specify the size of the message payload in bytes
                - `--timeout_duration_ms` to specify the duration in milliseconds to wait for a reply before retransmitting the message
                - `--initial_delay_s` to specify the delay in seconds before sending the first message
    2. For unicast:
        1. Run the latency_tester_secondary_node: `./latency_tester_secondary_node --interface {robot_wifi_interface} --unicast`
            - You may optionally also provide the following arguments:
                - `--runtime_dir` to specify the directory where log files are stored
                - `--listen_port` to specify the port on which the secondary node listens for messages.
                - `--send_port` to specify the port on which the secondary node sends messages
                - `--send_ip` to specify the IP address of the primary node to send replies to
        2. On a different terminal on the host computer, run the latency_tester_primary_node: `./tbots.py run latency_tester_primary_node -- --interface {host_interface} --unicast`
            - You may optionally also provide the following arguments:
                - `--runtime_dir` to specify the directory where log files are stored
                - `--listen_port` to specify the port on which the primary node listens for replies to messages. This port must match the `--send_port` argument provided to the secondary node.
                - `--send_port` to specify the port on which the primary node sends messages. This port must match the `--listen_port` argument provided to the secondary node.
                - `--send_ip` to specify the IP address of the secondary node to send messages to (`{robot_ip}`)
                - `--num_messages` to specify the number of messages to send
                - `--message_size_bytes` to specify the size of the message payload in bytes
                - `--timeout_duration_ms` to specify the duration in milliseconds to wait for a reply before retransmitting the message
                - `--initial_delay_s` to specify the delay in seconds before sending the first message
3. This tool can also be run with Tracy, a profiling tool, which provides some nice performance visualizations and histograms. To do so:
    1. Make sure Tracy has been installed. Run `./environment_setup/install_tracy.sh` to install Tracy.
    2. On a new terminal in the host computer run Tracy: `./tbots.py run tracy`
    3. When running the latency tester primary node, add the `--tracy` flag to the command before the `--`. For example: `./tbots.py run latency_tester_primary_node --tracy -- --interface {host_interface}`
    4. Tracy will allow you to select the binary to profile and provide detailed performance information after the tester has run.
