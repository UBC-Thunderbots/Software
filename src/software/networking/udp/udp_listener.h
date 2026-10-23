#pragma once

#include <array>
#include <atomic>
#include <boost/asio.hpp>
#include <functional>
#include <future>
#include <memory>
#include <string>

typedef std::function<void(const char*, const size_t&)> ReceiveCallback;

/**
 * Creates a UDP listener that can listen on a given port and interface.
 */
class UdpListener
{
   public:
    /**
     * Creates a UDP listener.
     *
     * @throws TbotsNetworkException if the listener could not be created
     *
     * @param io_context The io_context to use for the network communication resource
     * @param ip_address If multicast is true, this address is the multicast group to
     * join. Otherwise, this is the IP address of the local interface to listen on
     * @param port The port to listen on
     * @param interface The networking interface to listen on
     * @param multicast If true, the listener will join the multicast group given by
     * `ip_address`, otherwise it will listen on the local interface given by `ip_address`
     * and `interface`
     * @param receive_callback The callback to call when a new message is received
     */
    UdpListener(boost::asio::io_context& io_context, const std::string& ip_address,
                unsigned short port, const std::string& interface, bool multicast,
                ReceiveCallback receive_callback);

    /**
     * Creates a UDP listener that listens on the given port on all interfaces.
     *
     * @throws TbotsNetworkException if the listener could not be created
     *
     * @param io_context The io_context to use for the network communication resource
     * @param port The port to listen on
     * @param receive_callback The callback to call when a new message is received
     */
    UdpListener(boost::asio::io_context& io_context, const unsigned short port,
                ReceiveCallback receive_callback);

    /** Requests a stop without waiting for outstanding receive work to finish. */
    virtual ~UdpListener();

    /**
     * Closes the socket and waits until no callback can run again. Must not be called
     * from the io_context worker thread.
     */
    void close();

    /**
     * Requests a non-blocking stop; the worker closes the socket.
     */
    void requestStop();

   private:
    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;

    /**
     * Kept alive by pending handlers so the socket, buffer, and callback outlive the
     * UdpListener wrapper when necessary.
     */
    struct ReceiveState
    {
        ReceiveState(boost::asio::io_context& io_context,
                     ReceiveCallback receive_callback)
            : socket(io_context),
              receive_callback(std::move(receive_callback)),
              shutdown_future(shutdown_promise.get_future())
        {
        }

        boost::asio::ip::udp::socket socket;
        std::array<char, MAX_BUFFER_LENGTH> raw_received_data;
        boost::asio::ip::udp::endpoint sender_endpoint;
        ReceiveCallback receive_callback;

        // Worker-thread-only state.
        bool stopping = false;

        bool shutdown_signalled = false;

        std::atomic<bool> receive_in_flight{false};

        std::atomic<bool> stop_requested{false};

        std::promise<void> shutdown_promise;
        std::shared_future<void> shutdown_future;
    };

    static void startListen(const std::shared_ptr<ReceiveState>& state);

    static void handleDataReception(const std::shared_ptr<ReceiveState>& state,
                                    const boost::system::error_code& error,
                                    std::size_t bytes_transferred);

    static void maybeSignalShutdown(const std::shared_ptr<ReceiveState>& state);

    static void setupMulticast(boost::asio::ip::udp::socket& socket,
                               const boost::asio::ip::address& ip_address,
                               const std::string& listen_interface);

    std::shared_ptr<ReceiveState> state_;
};
