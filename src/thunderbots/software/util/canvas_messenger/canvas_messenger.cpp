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

void CanvasMessenger::publishAndClearAllLayers()
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(layers_map_lock);

    // Limit rate of the message publishing
    // Get the time right now
    const std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    const int64_t elapsed_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(now - time_last_published)
            .count();
    const double elapsed_ms = elapsed_ns / 1.0e6;

    // Do not do anything if the time passed hasn't been
    // long enough
    if (elapsed_ms < DESIRED_PERIOD_MS)
        return;

    // Send a payload per layer of messages
    for (const auto& layer_pair : this->layers_map)
    {
        // First is the layer number
        const uint8_t layer_number = (uint8_t)layer_pair.first;

        // Second is the vector that contains the sprites
        const std::vector<Sprite>& sprites = layer_pair.second;

        this->publishPayload(layer_number, sprites);
    }

    // Clear shapes in layers of current frame/tick
    this->clearAllLayers();

    // Update last published time
    time_last_published = now;
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

    // and publish
    this->publisher.publish(new_layer);
}

void CanvasMessenger::clearAllLayers()
{
    // Clears all sprite vector in all the layers
    for (auto& layer : this->layers_map)
    {
        layer.second.clear();
    }
}

void CanvasMessenger::clearLayer(Layer layer)
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(layers_map_lock);

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
    std::lock_guard<std::mutex> best_known_pass_lock(layers_map_lock);

    // We look if the layer exists
    if (this->layers_map.find(layer) == this->layers_map.end())
    {
        // If not, we initialize new key to empty vector pair
        this->layers_map[layer];
    }

    // and add the sprite to the layer vector
    this->layers_map[layer].emplace_back(sprite);
}

void CanvasMessenger::drawRectangle(Layer layer, Rectangle rectangle, Angle orientation,
                                    Color color)
{
    // We switch the width and height here because they're switched in the visualizer
    Sprite rectangle_sprite(0, rectangle.centre(), orientation, rectangle.width(),
                            rectangle.height(), color);

    drawSprite(layer, rectangle_sprite);
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
    drawBall(world.ball());
    drawField(world.field());
    drawTeam(world.friendlyTeam(), FRIENDLY_TEAM_COLOR);
    drawTeam(world.enemyTeam(), ENEMY_TEAM_COLOR);
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
    x.base = top_left_corner.x() * size_scaling_factor;
    payload.emplace_back(x.result[1]);
    payload.emplace_back(x.result[0]);

    Int16OrTwoInt8 y;
    // We negate y because we want positive y to be upwards in the visualizer, which uses
    // the graphics convention of +y downwards
    y.base = -top_left_corner.y() * size_scaling_factor;
    payload.emplace_back(y.result[1]);
    payload.emplace_back(y.result[0]);

    Int16OrTwoInt8 width;
    width.base = _width * size_scaling_factor;
    payload.emplace_back(width.result[1]);
    payload.emplace_back(width.result[0]);

    Int16OrTwoInt8 height;
    height.base = _height * size_scaling_factor;
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
