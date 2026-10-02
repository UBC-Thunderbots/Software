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

    /**
     * Destructor. Requests a stop of any outstanding receive work but does not block on
     * the io_context worker. Callers requiring a guarantee that no further callbacks run
     * after destruction must call close() first (ThreadedProtoUdpListener does this).
     */
    virtual ~UdpListener();

    /**
     * Cleans up all associated networking resources with this listener and blocks until
     * the listener is fully stopped and no user callback is running.
     *
     * This must not be called from the io_context worker thread that services this
     * listener, as it would wait on that same thread.
     */
    void close();

    /**
     * Requests that this listener stop receiving. Non-blocking; returns after the stop
     * has been scheduled on the io_context worker. The socket is closed once the worker
     * processes the request.
     */
    void requestStop();

   private:
    // The maximum buffer length for the raw data received from the network
    static constexpr unsigned int MAX_BUFFER_LENGTH = 9000;

    /**
     * Shared state referenced by every outstanding asynchronous receive handler, keeping
     * the socket, receive buffer and callback alive until the last pending handler
     * completes, independent of the lifetime of the UdpListener wrapper.
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

        // Set only on the io_context worker thread.
        bool stopping = false;

        // Set only on the io_context worker thread.
        bool shutdown_signalled = false;

        // Written by both the constructing thread and the worker thread, so it is atomic.
        std::atomic<bool> receive_in_flight{false};

        // Makes repeated and concurrent close()/requestStop() calls idempotent.
        std::atomic<bool> stop_requested{false};

        // Signalled on the worker thread once the listener is stopped and drained;
        // waited on externally by close().
        std::promise<void> shutdown_promise;
        std::shared_future<void> shutdown_future;
    };

    /**
     * Starts listening for data on the socket. Initiates a single asynchronous receive.
     */
    static void startListen(const std::shared_ptr<ReceiveState>& state);

    /**
     * Handles the reception of data from the network as well as any errors that may
     * occur before calling the user-provided callback.
     */
    static void handleDataReception(const std::shared_ptr<ReceiveState>& state,
                                    const boost::system::error_code& error,
                                    std::size_t bytes_transferred);

    /**
     * Signals shutdown completion if a stop has been requested and no receive remains
     * outstanding. Runs on the io_context worker thread.
     */
    static void maybeSignalShutdown(const std::shared_ptr<ReceiveState>& state);

    /**
     * Sets up multicast for the given ip_address and listen_interface.
     *
     * @throws TbotsNetworkException if the multicast group could not be joined
     */
    static void setupMulticast(boost::asio::ip::udp::socket& socket,
                               const boost::asio::ip::address& ip_address,
                               const std::string& listen_interface);

    std::shared_ptr<ReceiveState> state_;
};
