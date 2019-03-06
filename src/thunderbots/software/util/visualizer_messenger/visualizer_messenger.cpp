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

    void VisualizerMessenger::onConnection()
    {
        // The io_context is required for all I/O
        boost::asio::io_service ioc{1};

        // The acceptor receives incoming connections
        // TODO: make IP and port a configurable constant somewhere?
        auto const address = boost::asio::ip::address::from_string("127.0.0.1");
        auto const port    = static_cast<unsigned short>(std::atoi("9091"));
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

            LOG(INFO) << "Connection received." << std::endl;

            // Lock the current list of sockets
            ws_mutex.lock();

            // Save this new websocket connection
            ws_connections.emplace_back(std::move(websocket));

            // Unlock the current list of sockets
            ws_mutex.unlock();

            LOG(INFO) << "Connection added." << std::endl;

            // TODO: this is disgusting. We should not tie up all the cpu
            // Yield so that other things can run
            std::this_thread::yield();
        }
    }

    void VisualizerMessenger::initializeWebsocket()
    {
        ws_thread = std::thread([this]() { return onConnection(); });
    }

    const LayerMsgMap& VisualizerMessenger::getLayerMap() const
    {
        return this->layers_name_to_msg_map;
    }

    void VisualizerMessenger::publishAndClearLayers()
    {
        LOG(INFO) << "Starting shape data publishing." << std::endl;

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

        // Lock the list of current websockets
        ws_mutex.lock();

        std::vector<int32_t> payload;
        for (const std::pair<std::string, LayerMsg>& layer_msg_pair : getLayerMap())
        {
            for (auto shape : layer_msg_pair.second)
            {
                payload.insert(payload.end(), shape.begin(), shape.end());
            }
        }

        // Send all the shapes to all the websocket connections we have
        for (websocket::stream<tcp::socket>& websocket : ws_connections)
        {
            try
            {
                websocket.write(boost::asio::buffer(payload));
            }
            // TODO: non-generic exception type
            catch (std::exception const& e)
            {
                LOG(ERROR) << e.what() << std::endl;
            }
        }

        // UnLock the list of current websockets
        ws_mutex.unlock();

        // Clear shapes in layers of current frame/tick
        clearLayers();

        // Update last published time
        time_last_published = now;
    }

    void VisualizerMessenger::clearLayers()
    {
        // Clears all shape vector in all the layers
        for (auto& layer : this->layers_name_to_msg_map)
        {
            layer.second.clear();
        }
    }

    //    void VisualizerMessenger::drawEllipse(const std::string& layer, double cx,
    //    double cy,
    //                                          double r1, double r2, DrawStyle
    //                                          draw_style, DrawTransform draw_transform)
    //    {
    //        ShapeMsg new_shape;
    //        new_shape.type = "ellipse";
    //        new_shape.data.clear();
    //        new_shape.data.push_back(cx);
    //        new_shape.data.push_back(cy);
    //        new_shape.data.push_back(r1);
    //        new_shape.data.push_back(r2);
    //
    //        applyDrawStyleToMsg(new_shape, draw_style);
    //        applyDrawTransformToMsg(new_shape, draw_transform);
    //        addShapeToLayer(layer, new_shape);
    //    }

    void VisualizerMessenger::drawRect(const std::string& layer, double x, double y,
                                       double w, double h, DrawStyle draw_style,
                                       DrawTransform draw_transform)
    {
        std::vector<int32_t> raw = {
            2, int(x), int(y), int(w), int(h), draw_transform.rotation, 0xFF, 0xFF, 0xFF};

        addShapeToLayer(layer, raw);
        // ShapeMsg new_shape;
        // new_shape.type = "rect";
        // new_shape.data.clear();
        // new_shape.data.push_back(x);
        // new_shape.data.push_back(y);
        // new_shape.data.push_back(w);
        // new_shape.data.push_back(h);

        // applyDrawStyleToMsg(new_shape, draw_style);
        // applyDrawTransformToMsg(new_shape, draw_transform);
        // addShapeToLayer(layer, new_shape);
    }

    //    void VisualizerMessenger::drawPoly(const std::string& layer,
    //                                       std::vector<Point>& vertices,
    //                                       DrawStyle draw_style, DrawTransform
    //                                       draw_transform)
    //    {
    //        ShapeMsg new_shape;
    //        new_shape.type = "poly";
    //        new_shape.data.clear();
    //
    //        for (auto vertexIter = vertices.begin(); vertexIter != vertices.end();
    //             vertexIter++)
    //        {
    //            new_shape.data.push_back((*vertexIter).x);
    //            new_shape.data.push_back((*vertexIter).y);
    //        }
    //
    //        applyDrawStyleToMsg(new_shape, draw_style);
    //        applyDrawTransformToMsg(new_shape, draw_transform);
    //        addShapeToLayer(layer, new_shape);
    //    }

    //    void VisualizerMessenger::drawArc(const std::string& layer, double cx, double
    //    cy,
    //                                      double radius, double theta_start, double
    //                                      theta_end, DrawStyle draw_style, DrawTransform
    //                                      draw_transform)
    //    {
    //        ShapeMsg new_shape;
    //        new_shape.type = "arc";
    //        new_shape.data.clear();
    //        new_shape.data.push_back(cx);
    //        new_shape.data.push_back(cy);
    //        new_shape.data.push_back(radius);
    //        new_shape.data.push_back(theta_start);
    //        new_shape.data.push_back(theta_end);
    //
    //        applyDrawStyleToMsg(new_shape, draw_style);
    //        applyDrawTransformToMsg(new_shape, draw_transform);
    //        addShapeToLayer(layer, new_shape);
    //    }

    //    void VisualizerMessenger::drawLine(const std::string& layer, double x1, double
    //    y1,
    //                                       double x2, double y2, DrawStyle draw_style,
    //                                       DrawTransform draw_transform)
    //    {
    //        ShapeMsg new_shape;
    //        new_shape.type = "line";
    //        new_shape.data.clear();
    //        new_shape.data.push_back(x1);
    //        new_shape.data.push_back(y1);
    //        new_shape.data.push_back(x2);
    //        new_shape.data.push_back(y2);
    //
    //        applyDrawStyleToMsg(new_shape, draw_style);
    //        applyDrawTransformToMsg(new_shape, draw_transform);
    //        addShapeToLayer(layer, new_shape);
    //    }

    //    void VisualizerMessenger::applyDrawStyleToMsg(ShapeMsg& shape_msg, DrawStyle&
    //    style)
    //    {
    //        shape_msg.fill          = style.fill;
    //        shape_msg.stroke        = style.stroke;
    //        shape_msg.stroke_weight = style.stroke_weight;
    //    }
    //
    //    void VisualizerMessenger::applyDrawTransformToMsg(ShapeMsg& shape_msg,
    //                                                      DrawTransform& transform)
    //    {
    //        shape_msg.transform_rotation = transform.rotation;
    //        shape_msg.transform_scale    = transform.scale;
    //    }

}  // namespace Util
