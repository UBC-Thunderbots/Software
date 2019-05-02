#include "canvas_messenger.h"

#include <firmware/main/shared_util/constants.h>

#include "util/constants.h"

using namespace Util;

std::shared_ptr<CanvasMessenger> CanvasMessenger::getInstance()
{
    static std::shared_ptr<CanvasMessenger> canvas_messenger(new CanvasMessenger);
    return canvas_messenger;
}

void CanvasMessenger::initializePublisher(ros::NodeHandle node_handle)
{
    this->publisher = node_handle.advertise<thunderbots_msgs::CanvasLayer>(
        Util::Constants::VISUALIZER_DRAW_LAYER_TOPIC, BUFFER_SIZE);
}

void CanvasMessenger::publishAndClearLayer(Layer layer)
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> layers_map_lock(layers_map_mutex);

    // Get the time right now
    const std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();

    // Make sure the layer exists
    auto layer_pair = layers_map.find(layer);
    if (layer_pair != layers_map.end())
    {
        // First is the layer number
        const uint8_t layer_number = (uint8_t)layer_pair->first;

        // Second is the vector that contains the sprites
        const std::vector<Sprite>& sprites = layer_pair->second.sprites;

        const int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       now - layer_pair->second.time)
                                       .count();
        const double elapsed_ms = elapsed_ns / 1.0e6;

        // Publish the layer if enough time has passed since we
        // last published
        if (elapsed_ms >= DESIRED_PERIOD_MS)
        {
            // Publish the layer
            this->publishPayload(layer_number, sprites);

            // Update last published time
            layer_pair->second.time = now;
        }

        // Clear the layer
        layer_pair->second.sprites = {};
    }
}

void CanvasMessenger::publishPayload(uint8_t layer, std::vector<Sprite> sprites)
{
    std::vector<uint8_t> payload;

    // Add the layer number at the begining of the binary message
    payload.insert(payload.end(), layer);

    // Convert each sprite into a binary message
    for (Sprite& sprite : sprites)
    {
        std::vector<uint8_t> sprite_payload = sprite.serialize(PIXELS_PER_METER);
        payload.insert(payload.end(), sprite_payload.begin(), sprite_payload.end());
    }

    // Create a new layer message, add the binary data
    thunderbots_msgs::CanvasLayer new_layer;
    new_layer.data = payload;

    // and publish if we have a valid ROS publisher.
    // This check is important for cases where we're running this without running ROS,
    // such as in unit tests.
    if (publisher)
    {
        publisher->publish(new_layer);
    }
}

void CanvasMessenger::clearAllLayers()
{
    // Clears all sprite vector in all the layers
    for (auto& layer : this->layers_map)
    {
        layer.second.sprites.clear();
    }
}

void CanvasMessenger::clearLayer(Layer layer)
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> layers_map_lock(layers_map_mutex);

    if (layers_map.find(layer) != layers_map.end())
    {
        layers_map[layer] = {};
    }
}

void CanvasMessenger::drawSprite(Layer layer, Sprite sprite)
{
    // We simply add the sprite to the specified layer
    this->addSpriteToLayer(layer, sprite);
}

void CanvasMessenger::addSpriteToLayer(Layer layer, Sprite& sprite)
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> layers_map_lock(layers_map_mutex);

    // We look if the layer exists
    if (this->layers_map.find(layer) == this->layers_map.end())
    {
        // If not, we initialize new key to empty vector pair
        this->layers_map[layer];
    }

    // and add the sprite to the layer vector
    this->layers_map[layer].sprites.emplace_back(sprite);
}

void CanvasMessenger::drawRectangle(Layer layer, Rectangle rectangle, Angle orientation,
                                    Color color)
{
    // We switch the width and height here because they're switched in the visualizer
    Sprite rectangle_sprite(0, rectangle.centre(), orientation, rectangle.width(),
                            rectangle.height(), color);

    drawSprite(layer, rectangle_sprite);
}

void CanvasMessenger::drawGradient(Layer layer, std::function<double(Point)> valueAtPoint,
                                   const Rectangle& area, double min_val, double max_val,
                                   Color min_color, Color max_color, int points_per_meter)
{
    for (int i = 0; i < area.width() * points_per_meter; i++)
    {
        for (int j = 0; j < area.height() * points_per_meter; j++)
        {
            Point p = area.swCorner() +
                      Vector(0.5 / points_per_meter, 0.5 / points_per_meter) +
                      Vector(i / static_cast<double>(points_per_meter),
                             j / static_cast<double>(points_per_meter));

            // Get the value and clamp it appropriately
            double val_at_p = std::clamp(valueAtPoint(p), min_val, max_val);

            // Create the "pixel" in the gradient
            Rectangle block(p - Vector(0.5 / points_per_meter, 0.5 / points_per_meter),
                            p + Vector(0.5 / points_per_meter, 0.5 / points_per_meter));

            // Linearly interpolate the color
            Color color = {
                static_cast<uint8_t>((max_color.r - min_color.r) / (max_val - min_val) *
                                         (val_at_p - min_val) +
                                     min_color.r),
                static_cast<uint8_t>((max_color.g - min_color.g) / (max_val - min_val) *
                                         (val_at_p - min_val) +
                                     min_color.g),
                static_cast<uint8_t>((max_color.b - min_color.b) / (max_val - min_val) *
                                         (val_at_p - min_val) +
                                     min_color.b),
                static_cast<uint8_t>((max_color.a - min_color.a) / (max_val - min_val) *
                                         (val_at_p - min_val) +
                                     min_color.a)};
            drawRectangle(layer, block, Angle::zero(), color);
        }
    }
}

void CanvasMessenger::drawLine(Layer layer, Point p1, Point p2, double thickness,
                               Color color)
{
    // Since we're just repurposing the Rectangle sprite as a line, we need to figure
    // out where its center is
    Point line_center      = (p2 + p1) / 2;
    Angle line_orientation = (p2 - p1).orientation();
    double line_length     = (p2 - p1).len();

    Sprite rectangle_sprite(0, line_center, line_orientation, line_length, thickness,
                            color);

    drawSprite(layer, rectangle_sprite);
}

void CanvasMessenger::drawPoint(Layer layer, const Point& p, double radius, Color color)
{
    Sprite circle_sprite(
        // NOTE: Currently this uses texture ID zero, which is a rectangle, but
        // eventually we should use a proper circle texture
        0, p, Angle::zero(), radius * 2, radius * 2, color);
    drawSprite(layer, circle_sprite);
}

void CanvasMessenger::drawWorld(const World& world)
{
    // Draw the new layer
    drawBall(world.ball());
    publishAndClearLayer(Layer::BALL);

    drawField(world.field());
    publishAndClearLayer(Layer::STATIC_FEATURES);

    drawTeam(world.friendlyTeam(), FRIENDLY_TEAM_COLOR);
    drawTeam(world.enemyTeam(), ENEMY_TEAM_COLOR);
    publishAndClearLayer(Layer::ROBOTS);
}

void CanvasMessenger::drawBall(const Ball& ball)
{
    drawPoint(Layer::BALL, ball.position(), BALL_MAX_RADIUS_METERS * 2, BALL_COLOR);
}

void CanvasMessenger::drawField(Field field)
{
    // Draw the base of the field
    drawRectangle(Layer::STATIC_FEATURES,
                  Rectangle(field.enemyCornerNeg(), field.friendlyCornerPos()),
                  Angle::zero(), FIELD_COLOR);

    // Draw the defense areas
    drawRectangle(Layer::STATIC_FEATURES, field.enemyDefenseArea(), Angle::zero(),
                  DEFENSE_AREA_COLOR);
    drawRectangle(Layer::STATIC_FEATURES, field.friendlyDefenseArea(), Angle::zero(),
                  DEFENSE_AREA_COLOR);

    // Draw the center line
    drawLine(Layer::STATIC_FEATURES, {0, -field.width() / 2}, {0, field.width() / 2},
             0.05, FIELD_LINE_COLOR);

    // Draw a marker for the origin
    drawLine(Layer::STATIC_FEATURES, {-0.1, 0}, {0.1, 0}, 0.05, FIELD_LINE_COLOR);
}

void CanvasMessenger::drawTeam(const Team& team, CanvasMessenger::Color color)
{
    for (const Robot& robot : team.getAllRobots())
    {
        drawRobot(robot, color);
    }
}

void CanvasMessenger::drawRobot(Robot robot, CanvasMessenger::Color color)
{
    Rectangle robot_rectangle(
        robot.position() + Vector(ROBOT_MAX_RADIUS_METERS, ROBOT_MAX_RADIUS_METERS),
        robot.position() + Vector(-ROBOT_MAX_RADIUS_METERS, -ROBOT_MAX_RADIUS_METERS));
    drawRectangle(Layer::ROBOTS, robot_rectangle, robot.orientation(), color);
}

Point CanvasMessenger::Sprite::getTopLeftCorner()
{
    return _center + Vector(-_width / 2, _height / 2);
}

std::vector<uint8_t> CanvasMessenger::Sprite::serialize(int size_scaling_factor)
{
    std::vector<uint8_t> payload;

    payload.emplace_back(_texture);

    Point top_left_corner = getTopLeftCorner();

    // These are all 16 bits, we need to split them
    Int16OrTwoInt8 x;
    x.base = std::floor(top_left_corner.x() * size_scaling_factor);
    payload.emplace_back(x.result[1]);
    payload.emplace_back(x.result[0]);

    Int16OrTwoInt8 y;
    // We negate y because we want positive y to be upwards in the visualizer, which uses
    // the graphics convention of +y downwards
    y.base = std::floor(-top_left_corner.y() * size_scaling_factor);
    payload.emplace_back(y.result[1]);
    payload.emplace_back(y.result[0]);

    Int16OrTwoInt8 width;
    width.base = std::round(_width * size_scaling_factor);
    payload.emplace_back(width.result[1]);
    payload.emplace_back(width.result[0]);

    Int16OrTwoInt8 height;
    height.base = std::round(_height * size_scaling_factor);
    payload.emplace_back(height.result[1]);
    payload.emplace_back(height.result[0]);

    Int16OrTwoInt8 orientation;
    // We serialize to tenths of degrees, and also negate the orientation to match
    // our on-field conventions
    orientation.base = -_orientation.toDegrees() * 10;
    payload.emplace_back(orientation.result[1]);
    payload.emplace_back(orientation.result[0]);

    payload.emplace_back(_color.a);
    payload.emplace_back(_color.r);
    payload.emplace_back(_color.g);
    payload.emplace_back(_color.b);

    return payload;
}
