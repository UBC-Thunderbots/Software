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

#include "thunderbots_msgs/CanvasLayer.h"
#include "util/constants.h"

#include "ai/world/field.h"

namespace Util
{
    class CanvasMessenger
    {
       public:
        /**
         * Sprite is a struct that contains all the information
         * necessary to create a sprite. The default constructor
         * creates a 100x100 white rectangle sprite.
         */
        typedef struct Sprite
        {
            Sprite()
                : texture(0),
                  x(0.0),
                  y(0.0),
                  width(100.0),
                  height(100.0),
                  rotation(0.0),
                  opacity(255),
                  red(255),
                  green(255),
                  blue(255)
            {
            }

            uint8_t texture;
            int16_t x;
            int16_t y;
            int16_t width;
            int16_t height;
            int16_t rotation;
            uint8_t opacity;
            uint8_t red;
            uint8_t green;
            uint8_t blue;
        } Sprite;

       public:
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
         * Draw a point at a given location with a given radius
         *
         * @param p The point to draw
         * // TODO: Units for the radius???
         * @param radius The radius to draw the point with
         */
        void drawPoint(Point p, double radius);

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
        /**
         * Constructor; initializes an empty layers map then populates it
         */
        explicit CanvasMessenger() : layers_map(), publisher() {}

        void publishPayload(uint8_t layer, const std::vector<Sprite>& shapes);

        /**
         * Add sprite to layer
         * @param layer: The layer which the sprite is to be added to
         * @param sprite_data: The sprite data
         */
        void addSpriteToLayer(uint8_t layer, Sprite& sprite_data);

        /**
         * Draw a sprite onto a specific layer.
         *
         * @param layer: The layer number this shape is being drawn to
         * @param sprite: the sprite data to draw
         */
        void drawSprite(uint8_t layer, Sprite sprite);


        // layer to sprite data map
        std::map<uint8_t, std::vector<Sprite>> layers_map;
        ros::Publisher publisher;

        // Period in nanoseconds
        const double DESIRED_PERIOD_MS =
            1.0e3 / Util::Constants::DESIRED_CANVAS_MESSAGE_FREQ;

        // Number of messages we want our ROS publisher to buffer
        const int BUFFER_SIZE = 8;

        // Time point
        std::chrono::time_point<std::chrono::system_clock> time_last_published;
    };
}  // namespace Util
