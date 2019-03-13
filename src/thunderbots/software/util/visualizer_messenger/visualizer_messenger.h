/**
 * The Visualizer messenger is a singleton object that receives draw calls
 * to draw shapes such as ellipse and rectangle.
 *
 * Lasted edited by Muchen He on 2019-03-09
 */

#pragma once

#include <ros/ros.h>

#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "geom/point.h"
#include "util/constants.h"

// Forward declaration
namespace ros
{
    class NodeHandle;
    class Publisher;
}  // namespace ros

namespace Util
{
    using time_point = std::chrono::time_point<std::chrono::system_clock>;
    using websocket_connection_vector =
        std::vector<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>;

    class VisualizerMessenger
    {
       public:
        /**
         * ShapeStyle is a struct that packs the appearance properties that goes into a
         * shape message The default constructor contains the default value of the draw
         * style
         */
        typedef struct ShapeStyle
        {
            // Struct constructor
            ShapeStyle() : texture(0), tint(0xFFFFFFF) {}

            uint8_t texture;
            uint32_t tint;
        } ShapeStyle;

        typedef struct Shape
        {
            uint8_t texture;
            uint8_t flags;
            int16_t x;
            int16_t y;
            int16_t width;
            int16_t height;
            int16_t rotation;
            uint32_t tint;
        } Shape;
        using ShapeVector = std::vector<Shape>;
        using LayerMap    = std::map<uint8_t, ShapeVector>;

       public:
        /**
         * Getter of the singleton object.
         *
         * @return A shared pointer of the static instance
         */
        static std::shared_ptr<VisualizerMessenger> getInstance();

        /**
         * Initialize websocket for the visualizer
         */
        void initializeWebsocket();

        /**
         * Update call to the visualizer class
         *
         * Uses ROS publishers to publish layer messeges stored in the member layers map
         * and then clear the shapes in the layers in the member layers map
         */
        void publishAndClearLayers();

        // Drawing methods
        /**
         * Request a message to draw a ellipse shape. The origin is the center of the
         * circle.
         *
         * @param layer: The layer number this shape is being drawn to
         * @param cx: Ellipse's center X             [mm]
         * @param cy: Ellipse's center Y             [mm]
         * @param r1: Ellipse's horizontal radius    [mm]
         * @param r2: Ellipse's vertical radius      [mm]
         * @param rotation: Shape's rotation         [deg]
         * @param style: Shape style struct
         */
        void drawEllipse(uint8_t layer, uint16_t cx, uint16_t cy, int16_t r1, int16_t r2,
                         int16_t rotation, ShapeStyle style = ShapeStyle());

        /**
         * Request a message to draw a rectangle shape. The rectangle has origin on the
         * upper left corner, this is also the point specified in the parameter.
         *
         * @param layer: The layer number this shape is being drawn to
         * @param x: Rectangle's starting point X   [mm]
         * @param y: Rectangle's starting point Y   [mm]
         * @param w: Rectangle width                [mm]
         * @param h: Rectangle height               [mm]
         * @param rotation: Shape's rotation        [deg]
         * @param style: Shape style struct
         */
        void drawRect(uint8_t layer, int16_t x, int16_t y, int16_t w, int16_t h,
                      int16_t rotation, ShapeStyle style = ShapeStyle());

        /**
         * Request a message to draw a line. The origin is the first point
         *
         * @param layer: The layer number this shape is being drawn to
         * @param x1: Starting point X              [mm]
         * @param y1: Starting point Y              [mm]
         * @param x2: Ending point X                [mm]
         * @param y2: Ending point Y                [mm]
         * @param rotation: Shape's rotation        [deg]
         * @param style: Shape style struct
         */
        void drawLine(uint8_t layer, int16_t x1, int16_t y1, int16_t x2, int16_t y2,
                      uint8_t width, ShapeStyle style = ShapeStyle());

       private:
        /**
         * Constructor; initializes an empty layers map then populates it
         */
        explicit VisualizerMessenger()
            : layer_shapes_map(),
              time_last_published(),
              websocket_thread(),
              websocket_mutex(),
              websocket_connections()
        {
        }

        /**
         * Handles any connections
         */
        void receiveWebsocketConnections();

        /**
         * Sends a layer of shape data through websocket
         * @param layer: The layer number to be "published" on websocket
         * @param shapes: A const reference of the vector of shapes that belong to the
         * layer
         */
        void publishPayload(uint8_t layer, const ShapeVector& shapes);

        /**
         * Clears the shapes array in the layer to shape map for this frame
         */
        void clearShapes();

        /**
         * Add shape to layer
         * @param layer: The layer which the shape is to be added to
         * @param shape: The shape
         */
        void addShapeToLayer(uint8_t layer, Shape& shape);

        /**
         * Pack two 8-bit data to a single 16-bit data
         * @param in1: first data
         * @param in2: second data
         * @return 16 bit concatenated data <first><second>
         */
        inline uint16_t pack16BitData(uint8_t in1, uint8_t in2)
        {
            return ((static_cast<uint16_t>(in1)) << 8) | (in2 & 0x00FF);
        }

        // Period in nanoseconds
        const double DESIRED_PERIOD_MS =
            1.0e3 / Util::Constants::DESIRED_VISUALIZER_MESSAGE_FREQ;

        // Time point
        time_point time_last_published;

        // Thread on which we watch for websocket connections
        std::thread websocket_thread;

        // Mutex on the list of current websocket connections
        std::mutex websocket_mutex;

        // All the current websocket connections we have
        websocket_connection_vector websocket_connections;

        // A map that contains the layers and shapes of this frame
        LayerMap layer_shapes_map;
    };

}  // namespace Util
