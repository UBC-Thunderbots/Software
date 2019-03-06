/**
 * The Visualizer messenger is a singleton object that receives draw calls
 * to draw shapes such as ellipse and rectangle.
 *
 * The messenger constructs and queues the shapes into shape messages and layer
 * messages to be sent via ROS visualizer topics (VISUALIZER_DRAW_LAYER_TOPIC)
 *
 * Lasted edited by Muchen He on 2019-01-30
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
// #include "thunderbots_msgs/DrawLayer.h"
// #include "thunderbots_msgs/DrawShape.h"
#include "util/constants.h"


// Forward declaration
namespace ros
{
    class NodeHandle;
    class Publisher;
}  // namespace ros

namespace Util
{
    using ShapeMsg    = std::vector<int32_t>;
    using LayerMsg    = std::vector<ShapeMsg>;
    using LayerMsgMap = std::map<std::string, LayerMsg>;
    using time_point  = std::chrono::time_point<std::chrono::system_clock>;

    class VisualizerMessenger
    {
       public:
        /**
         * DrawStyle is a struct that packs the appearance properties that goes into a
         * shape message The default constructor contains the default value of the draw
         * style
         */
        typedef struct DrawStyle
        {
            // Struct constructor
            DrawStyle() : fill("white"), stroke("black"), stroke_weight(1) {}

            std::string fill;
            std::string stroke;
            uint8_t stroke_weight;
        } DrawStyle;

        /**
         * DrawTransform is a struct that packs the transform properties that goes into a
         * shape message The default constructor contains the default value of the
         * transformation
         * 
         * Rotation is in degrees
         */
        typedef struct DrawTransform
        {
            DrawTransform() : rotation(0), scale(1.0) {}

            int rotation;
            double scale;
        } DrawTransform;

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
         * Get a constant reference of the map of existing layers
         *
         * @return Constant reference of the layer message map
         */
        const LayerMsgMap& getLayerMap() const;

        /**
         * Update call to the visualizer class
         *
         * Uses ROS publishers to publish layer messeges stored in the member layers map
         * and then clear the shapes in the layers in the member layers map
         */
        void publishAndClearLayers();

        /**
         * Clears the content (shape vector) in the layer message upon method call
         */
        void clearLayers();

        // Drawing methods
        ///**
        // * Request a message to draw a ellipse shape. The origin is the center of the
        // * circle.
        // *
        // * @param layer: The layer name this shape is being drawn to
        // * @param cx: Ellipse's center X
        // * @param cy: Ellipse's center Y
        // * @param r1: Ellipse's horizontal radius
        // * @param r2: Ellipse's vertical radius
        // * @param draw_style: the drawing style of the shape
        // * @param draw_transform: the transformation of the shape
        // */
        // void drawEllipse(const std::string& layer, double cx, double cy, double r1,
        //                 double r2, DrawStyle draw_style = DrawStyle(),
        //                 DrawTransform draw_transform = DrawTransform());

        /**
         * Request a message to draw a rectangle shape. The rectangle has origin on the
         * upper left corner, this is also the point specified in the parameter.
         *
         * @param layer: The layer name this shape is being drawn to
         * @param x: Rectangle's starting point X
         * @param y: Rectangle's starting point Y
         * @param w: Rectangle width
         * @param h: Rectangle height
         * @param draw_style: the drawing style of the shape
         * @param draw_transform: the transformation of the shape
         */
        void drawRect(const std::string& layer, double x, double y, double w, double h,
                      DrawStyle draw_style         = DrawStyle(),
                      DrawTransform draw_transform = DrawTransform());

        ///**
        // * Request a message to draw a polygon shape. The origin is the
        // * first vertex passed in.
        // *
        // * @param layer: The layer name this shape is being drawn to
        // * @param vertices: A vector of Points that specifies x and y of vertices
        // * @param draw_style: the drawing style of the shape
        // * @param draw_transform: the transformation of the shape
        // */
        // void drawPoly(const std::string& layer, std::vector<Point>& vertices,
        //              DrawStyle draw_style         = DrawStyle(),
        //              DrawTransform draw_transform = DrawTransform());

        ///**
        // * Request a message to draw an arc. The origin is the center point
        // *
        // * @param layer: The layer name this shape is being drawn to
        // * @param cx: Arc's center point X
        // * @param cy: Arc's center point Y
        // * @param radius: Arc's radius
        // * @param theta_start: The starting angle of the arc in rad (where 0 is pointed
        // to
        // * +X horizontal heading, and PI/2 is south)
        // * @param theta_end: The ending angle of the arc in rad
        // * @param draw_style: the drawing style of the shape
        // * @param draw_transform: the transformation of the shape
        // */
        // void drawArc(const std::string& layer, double cx, double cy, double radius,
        //             double theta_start, double theta_end,
        //             DrawStyle draw_style         = DrawStyle(),
        //             DrawTransform draw_transform = DrawTransform());

        ///**
        // * Request a message to draw a line. The origin is the first point
        // *
        // * @param layer: The layer name this shape is being drawn to
        // * @param x1: Starting point X
        // * @param y1: Starting point Y
        // * @param x2: Ending point X
        // * @param y2: Ending point Y
        // * @param draw_style: the drawing style of the shape
        // * @param draw_transform: the transformation of the shape
        // */
        // void drawLine(const std::string& layer, double x1, double y1, double x2,
        //              double y2, DrawStyle draw_style = DrawStyle(),
        //              DrawTransform draw_transform = DrawTransform());

       private:
        /**
         * Constructor; initializes an empty layers map then populates it
         */
        explicit VisualizerMessenger()
            : layers_name_to_msg_map(),
              time_last_published(),
              ws_thread(),
              ws_mutex(),
              ws_connections()
        {
        }

        /**
         * Populates the layers map with some pre-defined layers
         */
        void buildLayers();

        /**
         * Helper function for the shape draw methods that copies the attributes in
         * DrawStyle struct into the actual shape message
         *
         * @param shape_msg: Reference to the shape message
         * @param style: The DrawStyle to be extracted
         */
        void applyDrawStyleToMsg(ShapeMsg& shape_msg, DrawStyle& style);

        /**
         * Helper function for the shape draw methods that copies the attributes in
         * DrawTransform struct into the actual shape message
         *
         * @param shape_msg: Reference to the shape message
         * @param transform: The DrawTransform struct to be extracted
         */
        void applyDrawTransformToMsg(ShapeMsg& shape_msg, DrawTransform& transform);

        /**
         * Helper function that checks if the layer exists,
         * and push the shape message to the corresponding layer's shape vector.
         *
         * @param layer: The name of the layer to push to
         * @param shape: The shape message
         */
        void addShapeToLayer(const std::string& layer, std::vector<int32_t>& shape);

        /**
         * Handles any connections
         */
        void onConnection();

        // string to LayerMsg map
        LayerMsgMap layers_name_to_msg_map;

        // Period in nanoseconds
        const double DESIRED_PERIOD_MS = 1.0e3 / Util::Constants::DESIRED_VISUALIZER_MESSAGE_FREQ;

        // Time point
        time_point time_last_published;

        // Thread on which we watch for websocket connections
        std::thread ws_thread;

        // Mutex on the list of current websocket connections
        std::mutex ws_mutex;

        // All the current websocket connections we have
        using websocket_connection_vector = std::vector<boost::beast::websocket::stream<boost::asio::ip::tcp::socket>>;
        websocket_connection_vector ws_connections;
    };

}  // namespace Util
