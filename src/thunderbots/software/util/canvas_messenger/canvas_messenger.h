/**
 * The Canvas Messenger is a singleton object that receives draw calls
 * to draw various shapes.
 *
 * The singleton constructs a binary representation of all the sprites
 * to be drawn for a particular layer and sends it to the visualizer via
 * ROS messages.
 */

#pragma once

#include <ros/ros.h>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "thunderbots_msgs/CanvasLayer.h"
#include "util/constants.h"

#include "ai/world/field.h"

namespace Util
{

    class CanvasMessenger
    {
       public:
        enum class Layer{
            STATIC_FEATURES,
            ROBOTS_AND_BALL
        };

        struct Color {
            // Red value of the color
            uint8_t r;
            // Green value of the color
            uint8_t g;
            // Blue value of the color
            uint8_t b;
            // Alpha (transparency) value of the color
            uint8_t a;
        };

        // TODO: Update this comment
        /**
         * Sprite is a struct that contains all the information
         * necessary to create a sprite. The default constructor
         * creates a 100x100 white rectangle sprite.
         */
        class Sprite {
        public:
            // delete the default constructor
            Sprite() = delete;

            /**
             * Create a Sprite
             * @param texture The texture id. On the visualizer side this will determine
             *        what sprite this is (circle, rectangle, robot, etc.)
             * @param center The center of the sprite
             * @param orientation The orientation of the sprite
             * @param width The width of the sprite (in meters, will be translated to pixels)
             * @param height The height of the sprite (in meters, will be translated to pixels)
             * @param color The color of the sprite
             */
            Sprite(uint8_t texture, const Point& center, const Angle& orientation, double width, double height, Color color)
                    : _texture(texture),
                      _center(center),
                      _orientation(orientation),
                      _color(color){}

                      /**
                       * Get the top left corner of the sprite
                       *
                       * @return the top left corner of the sprite
                       */
                      Point getTopLeftCorner();

            /**
             * Get the serialized form of this sprite
             *
             * This is the form that is streamed over to the web interface
             *
             * @return the serialized form of this sprite
             */
            std::vector<uint8_t> serialize();

        private:
            // TODO: Comment these
            uint8_t _texture;
            Point _center;
            double _width;
            double _height;
            Angle _orientation;
            Color _color;
        };

        /**
         * Getter of the singleton object.
         *
         * @return A shared pointer of the static instance
         */
        static std::shared_ptr<CanvasMessenger> getInstance();

        void initializePublisher(ros::NodeHandle node_handle);

        /**
         * Uses ROS publishers to publish sprite data for each layer and
         * then clears all layer data.
         */
        void publishAndClearLayers();

        /**
         * Clears all sprite data for all layers
         */
        void clearLayers();

        /**
         * Draw a rectangle on the given layer
         *
         * @param layer The layer to draw the rectangle on
         * @param rectangle The rectangle to draw (units are in meters)
         * @param color The color the rectangle should be
         */
        void drawRectangle(Layer layer, Rectangle rectangle, Angle orientation, Color color);

        /**
         * Draw a point at a given location with a given radius
         *
         * @param p The point to draw
         * // TODO: Units for the radius???
         * @param radius The radius to draw the point with
         */
        void drawPoint(Point p, double radius, int r, int g, int b, int opacity);

        /**
         * Draw the given field
         * @param field
         */
        void drawField(const Field& field);

       private:

        /**
         * Union used to convert a int16_t into two uint8_t
         */
        union Int16OrTwoInt8 {
            int16_t base;
            uint8_t result[2];
        };

        // The number of pixels per meter
        static const int PIXELS_PER_METER = 100;

        /**
         * Constructor; initializes an empty layers map then populates it
         */
        explicit CanvasMessenger() : layers_map(), publisher() {}

        void publishPayload(uint8_t layer, std::vector<Sprite> shapes);

        /**
         * Add sprite to layer
         * @param layer: The layer which the sprite is to be added to
         * @param sprite: The sprite data
         */
        void addSpriteToLayer(Layer layer, Sprite &sprite);

        /**
         * Draw a sprite onto a specific layer.
         *
         * @param layer: The layer number this shape is being drawn to
         * @param sprite: the sprite data to draw
         */
        void drawSprite(Layer layer, Sprite sprite);


        // layer to sprite data map
        std::map<Layer, std::vector<Sprite>> layers_map;
        ros::Publisher publisher;

        // Period in nanoseconds
        const double DESIRED_PERIOD_MS =
            1.0e3 / Util::Constants::DESIRED_CANVAS_MESSAGE_FREQ;

        // Number of messages we want our ROS publisher to buffer
        const int BUFFER_SIZE = 8;

        // Time point
        std::chrono::time_point<std::chrono::system_clock> time_last_published;

        // The mutex for the layers
        std::mutex layers_lock;
    };
}  // namespace Util
