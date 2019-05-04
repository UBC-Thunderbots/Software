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
#include <mutex>
#include <string>
#include <vector>

#include "ai/world/ball.h"
#include "ai/world/field.h"
#include "ai/world/robot.h"
#include "ai/world/world.h"
#include "thunderbots_msgs/CanvasLayer.h"
#include "util/constants.h"

namespace Util
{
    /**
     * This class provides an interface for drawing on the "Canvas" area of the visualizer
     *
     * All public methods are (and must be) thread safe, so please be careful when
     * exposing more public functionality
     */
    class CanvasMessenger
    {
       public:
        enum class Layer
        {
            PASS_GENERATION,
            BALL,
            ROBOTS,
            STATIC_FEATURES,
        };

        struct Color
        {
            // Red value of the color
            uint8_t r;
            // Green value of the color
            uint8_t g;
            // Blue value of the color
            uint8_t b;
            // Alpha (transparency) value of the color
            uint8_t a;
        };

        /**
         * Sprite is a struct that contains all the information
         * necessary to create a sprite.
         */
        class Sprite
        {
           public:
            // delete the default constructor
            Sprite() = delete;

            /**
             * Create a Sprite
             * @param texture The texture id. On the visualizer side this will determine
             *        what sprite this is (circle, rectangle, robot, etc.)
             * @param center The center of the sprite
             * @param orientation The orientation of the sprite
             * @param width The width of the sprite (in meters, will be translated to
             * pixels)
             * @param height The height of the sprite (in meters, will be translated to
             * pixels)
             * @param color The color of the sprite
             */
            Sprite(uint8_t texture, const Point &center, const Angle &orientation,
                   double width, double height, Color color)
                : _texture(texture),
                  _center(center),
                  _orientation(orientation),
                  _color(color),
                  _width(width),
                  _height(height)
            {
            }

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
            std::vector<uint8_t> serialize(int size_scaling_factor);

           private:
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
        void publishAndClearLayer(Layer layer);

        /**
         * Clear the given layer
         * @param layer The layer to clear
         */
        void clearLayer(Layer layer);

        /**
         * Draw a rectangle on the given layer
         *
         * @param layer The layer to draw the rectangle on
         * @param rectangle The rectangle to draw (units are in meters)
         * @param color The color the rectangle should be
         */
        void drawRectangle(Layer layer, Rectangle rectangle, Angle orientation,
                           Color color);

        /**
         * Draw a straight line from p1 to p2
         *
         * @param layer The layer to draw the line on
         * @param p1 The start point of the line
         * @param p2 The end point of the line
         * @param thickness The thickness of the line (units are in meters)
         * @param color The color of the line
         */
        void drawLine(Layer layer, Point p1, Point p2, double thickness, Color color);

        /**
         * Draw a point at a given location with a given radius
         *
         * @param p The point to draw
         * @param radius The radius to draw the point with
         */
        void drawPoint(Layer layer, const Point &p, double radius, Color color);

        /**
         * Draw a gradient created by the given function
         *
         * The color of the gradient will be determined by linear interpolation between
         * the given minimum and the maximum colors.
         *
         * @param layer The layer to draw the gradient on
         * @param valueAtPoint The function representing the gradient. This will be
         *                     called at points over the given area to get the value
         *                     to plot.
         * @param area The area over which to render the gradient
         * @param points_per_meter The number of points in a meter. Setting this to higher
         *                         values will give a higher resolution gradient, but be
         *                         wary, it increase the number of points at an n^2 rate
         * @param min_val The minimum value we expect `f` to return (values below this
         *                will automatically be clamped to this)
         * @param max_val The maximum value we expect `f` to return (values above this
         *                will automatically be clamped to this)
         * @param min_color The color for the minimum value
         * @param max_color The color for the maximum value
         */
        void drawGradient(Layer layer, std::function<double(Point)> valueAtPoint,
                          const Rectangle &area, double min_val, double max_val,
                          Color min_color, Color max_color, int points_per_meter);

        /**
         * Draw the given World
         *
         * This will draw all parts of the world over multiple layers
         *
         * @param world The world to draw
         */
        void drawWorld(const World &world);

        /**
         * Draw the given ball
         *
         * @param ball The ball to draw
         */
        void drawBall(const Ball &ball);

        /**
         * Draw the given field
         *
         * @param field The field to draw
         */
        void drawField(Field field);

        /**
         * Draw the given team
         *
         * @param team The team to draw
         * @param color The color to draw the team with
         */
        void drawTeam(const Team &team, Color color);

        /**
         * Draw the given robot
         *
         * @param robot The robot to draw
         * @param color The color of the robot
         */
        void drawRobot(Robot robot, Color color);

       private:
        /**
         * Union used to convert a int16_t into two uint8_t
         */
        union Int16OrTwoInt8 {
            int16_t base;
            uint8_t result[2];
        };

        /**
         * Struct that holds some sprites and a time
         */
        struct SpritesAndTime
        {
            std::vector<Sprite> sprites;
            std::chrono::time_point<std::chrono::system_clock> time;
        };

        // The number of pixels per meter
        static const int PIXELS_PER_METER = 2000;

        // Colors
        static constexpr Color FIELD_COLOR         = {0, 153, 0, 255};
        static constexpr Color DEFENSE_AREA_COLOR  = {242, 242, 242, 255};
        static constexpr Color FIELD_LINE_COLOR    = {242, 242, 242, 255};
        static constexpr Color BALL_COLOR          = {255, 153, 0, 255};
        static constexpr Color FRIENDLY_TEAM_COLOR = {230, 230, 0, 255};
        static constexpr Color ENEMY_TEAM_COLOR    = {0, 230, 230, 255};

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

        /**
         * Clears all sprite data for all layers
         */
        void clearAllLayers();

        std::optional<ros::Publisher> publisher;

        // Period in nanoseconds
        const double DESIRED_PERIOD_MS =
            1.0e3 / Util::Constants::DESIRED_CANVAS_MESSAGE_FREQ;

        // Number of messages we want our ROS publisher to buffer
        const int BUFFER_SIZE = 8;

        // Time point
        std::chrono::time_point<std::chrono::system_clock> time_last_published;

        // The mutex for the layers
        std::mutex layers_map_mutex;

        // layer to sprites/time last published map
        std::map<Layer, SpritesAndTime> layers_map;
    };
}  // namespace Util
