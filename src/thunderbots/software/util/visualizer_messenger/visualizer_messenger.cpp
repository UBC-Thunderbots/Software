#include "visualizer_messenger.h"

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>

#include "util/constants.h"
#include "util/logger/init.h"


using tcp           = boost::asio::ip::tcp;     // from <boost/asio/ip/tcp.hpp>
namespace websocket = boost::beast::websocket;  // from <boost/beast/websocket.hpp>

namespace Util
{
    std::shared_ptr<VisualizerMessenger> VisualizerMessenger::getInstance()
    {
        static std::shared_ptr<VisualizerMessenger> vm(new VisualizerMessenger);
        return vm;
    }

    void VisualizerMessenger::receiveWebsocketConnections()
    {
        // The io_context is required for all I/O
        boost::asio::io_service ioc{1};

        // The acceptor receives incoming connections
        auto const address = boost::asio::ip::address::from_string(
            Util::Constants::VISUALIZER_WEBSOCKET_ADDRESS);
        auto const port =
            static_cast<unsigned short>(Util::Constants::VISUALIZER_WEBSOCKET_PORT);
        tcp::acceptor acceptor{ioc, {address, port}};
        for (;;)
        {
            // This will receive the new connection
            tcp::socket socket{ioc};

            // Block until we get a connection
            acceptor.accept(socket);

            // Move the connection to a websocket stream
            websocket::stream<tcp::socket> websocket(std::move(socket));

            // Accept the websocket handshake
            websocket.accept();

            // Set the websocket to talk in binary
            websocket.binary(true);

            LOG(INFO) << "Visualizer websocket connection received." << std::endl;

            // Lock the current list of sockets
            websocket_mutex.lock();

            // Save this new websocket connection
            websocket_connections.emplace_back(std::move(websocket));

            // Unlock the current list of sockets
            websocket_mutex.unlock();

            LOG(INFO) << "Visualizer websocket connection added." << std::endl;
        }
    }

    void VisualizerMessenger::initializeWebsocket()
    {
        websocket_thread =
            std::thread([this]() { return receiveWebsocketConnections(); });
    }

    void VisualizerMessenger::publishAndClearLayers()
    {
        // Take ownership of list of websockets for the duration of this function
        std::lock_guard<std::mutex> websocket_list_lock(websocket_mutex);

        // Limit rate of the message publishing
        // Get the time right now
        const time_point now     = std::chrono::system_clock::now();
        const int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       now - time_last_published)
                                       .count();
        const double elapsed_ms = elapsed_ns / 1.0e6;

        // Do not do anything if the time passed hasn't been
        // long enough
        if (elapsed_ms < DESIRED_PERIOD_MS)
            return;

        // Send a payload per layer of messages
        for (const std::pair<uint8_t, ShapeVector>& layer_pair : this->layer_shapes_map)
        {
            // First is the layer number
            const uint8_t layer_number = layer_pair.first;

            // Second is the vector that contains the shapes
            const ShapeVector& shapes = layer_pair.second;

            // Only send the non-empty layers
            if (!layer_pair.second.empty())
            {
                this->publishPayload(layer_number, shapes);
            }
        }

        // Clear shapes in layers of current frame/tick
        this->clearShapes();

        // Update last published time
        time_last_published = now;

        LOG(DEBUG) << "Published shapes" << std::endl;
    }

    void VisualizerMessenger::publishPayload(uint8_t layer, const ShapeVector& shapes)
    {
        // Payload is a list of int32s
        std::vector<int16_t> payload;

        // Add layer number and layer flags that prepend shape data
        const uint8_t layer_flags = 0x00;
        payload.insert(payload.end(), pack16BitData(layer, layer_flags));

        // Unpack all the shape data and make a payload
        for (const Shape& shape : shapes)
        {
            // Packing texture (8 bit) and flag (8 bit) data into a single 16 bit int
            payload.insert(payload.end(), pack16BitData(shape.texture, shape.flags));

            // These are all 16 bits
            payload.insert(payload.end(), shape.x);
            payload.insert(payload.end(), shape.y);
            payload.insert(payload.end(), shape.width);
            payload.insert(payload.end(), shape.height);
            payload.insert(payload.end(), shape.rotation);

            // Split the 32 bit data into two parts
            payload.insert(payload.end(), static_cast<uint16_t>(shape.tint >> 16));
            payload.insert(payload.end(), static_cast<uint16_t>(shape.tint & 0xFF));
        }

        // Send all the shapes to all the websocket connections we have
        for (websocket::stream<tcp::socket>& websocket : websocket_connections)
        {
            try
            {
                websocket.write(boost::asio::buffer(payload));
            }
            // TODO: non-generic exception type
            catch (std::exception const& e)
            {
                LOG(WARNING) << e.what() << std::endl;
            }
        }
    }

    void VisualizerMessenger::clearShapes()
    {
        for (auto& layer : this->layer_shapes_map)
        {
            layer.second.clear();
        }
    }

    void VisualizerMessenger::drawEllipse(uint8_t layer, uint16_t cx, uint16_t cy,
                                          int16_t r1, int16_t r2, int16_t rotation,
                                          ShapeStyle style)
    {
        const uint8_t texture = style.texture;
        const uint32_t tint   = style.tint;

        Shape new_shape;
        new_shape.texture = texture;

        // Since x and y of the shape definition is top left, we need to shift to respect
        // the center of the ellipse
        new_shape.x = cx - r1;
        new_shape.y = cy - r2;

        new_shape.width  = r1 * 2;
        new_shape.height = r2 * 2;

        new_shape.rotation = rotation;
        new_shape.tint     = tint;

        this->addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::drawRect(uint8_t layer, int16_t x, int16_t y, int16_t w,
                                       int16_t h, int16_t rotation, ShapeStyle style)
    {
        const uint8_t texture = style.texture;
        const uint32_t tint   = style.tint;

        Shape new_shape;
        new_shape.texture  = texture;
        new_shape.x        = x;
        new_shape.y        = y;
        new_shape.width    = w;
        new_shape.height   = h;
        new_shape.rotation = rotation;
        new_shape.tint     = tint;

        this->addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::drawLine(uint8_t layer, int16_t x1, int16_t y1, int16_t x2,
                                       int16_t y2, uint8_t width, ShapeStyle style)
    {
        const uint8_t texture = style.texture;
        const uint32_t tint   = style.tint;

        const int16_t dx     = x2 - x1;
        const int16_t dy     = y2 - y1;
        const int16_t length = static_cast<int16_t>(sqrt(dx * dx + dy * dy));
        const int16_t angle  = static_cast<int16_t>(atan2(dy, dx));

        Shape new_shape;
        new_shape.texture  = texture;
        new_shape.x        = x1;
        new_shape.y        = y1;
        new_shape.width    = width;
        new_shape.height   = length;
        new_shape.rotation = angle;
        new_shape.tint     = tint;

        this->addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::addShapeToLayer(uint8_t layer, Shape& shape)
    {
        if (this->layer_shapes_map.find(layer) != this->layer_shapes_map.end())
        {
            this->layer_shapes_map[layer].emplace_back(shape);
        }
        else
        {
            LOG(WARNING) << "Referenced layer (" << layer << ") is undefined."
                         << std::endl;
        }
    }

}  // namespace Util
