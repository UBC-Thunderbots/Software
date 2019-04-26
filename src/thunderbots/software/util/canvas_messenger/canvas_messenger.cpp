#include "canvas_messenger.h"

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

void CanvasMessenger::publishAndClearLayers()
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(layers_lock);

    // Limit rate of the message publishing
    // Get the time right now
    const std::chrono::time_point<std::chrono::system_clock> now =
        std::chrono::system_clock::now();
    const int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                   now - time_last_published)
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
        const uint8_t layer_number = layer_pair.first;

        // Second is the vector that contains the sprites
        const std::vector<Sprite>& sprites = layer_pair.second;

        // Only send the non-empty layers
//            if (!layer_pair.second.empty())
//            {
            this->publishPayload(layer_number, sprites);
//            }
    }

    // Clear shapes in layers of current frame/tick
    this->clearLayers();

    // Update last published time
    time_last_published = now;
}

void CanvasMessenger::publishPayload(uint8_t layer,
                                     std::vector<Sprite> sprites)
{
    std::vector<uint8_t> payload;

    // Add the layer number at the begining of the binary message
    payload.insert(payload.end(), layer);

    // Convert each sprite into a binary message
    for (Sprite& sprite : sprites)
    {
        std::vector<uint8_t> sprite_payload = sprite.serialize();
        payload.insert(payload.end(), sprite_payload.begin(), sprite_payload.end());
    }

    // Create a new layer message, add the binary data
    thunderbots_msgs::CanvasLayer new_layer;
    new_layer.data = payload;

    // and publish
    this->publisher.publish(new_layer);
}

void CanvasMessenger::clearLayers()
{
    // Clears all sprite vector in all the layers
    for (auto& layer : this->layers_map)
    {
        layer.second.clear();
//            Util::CanvasMessenger::getInstance()->drawPoint(Point(), 0.5, 255, 0, 0);
    }
}


void CanvasMessenger::drawSprite(Layer layer, Sprite sprite)
{
    // We simply add the sprite to the specified layer
    this->addSpriteToLayer(layer, sprite);
}

void CanvasMessenger::addSpriteToLayer(Layer layer, Sprite &sprite)
{
    // Take ownership of the layers for the duration of this function
    std::lock_guard<std::mutex> best_known_pass_lock(layers_lock);

    // We look if the layer exists
    if (this->layers_map.find(layer) == this->layers_map.end())
    {
        // If not, we initialize new key to empty vector pair
        this->layers_map[layer];
    }

    // and add the sprite to the layer vector
    this->layers_map[layer].emplace_back(sprite);
}

void CanvasMessenger::drawRectangle(Layer layer, Rectangle rectangle, Angle orientation, Color color) {
    Sprite rectangle_sprite(0, rectangle.centre(), orientation, rectangle.width(), rectangle.height(), color);

    drawSprite(layer, rectangle_sprite);
}

//void CanvasMessenger::drawPoint(Point p, double radius, int r, int g, int b,
//                                int opacity) {
//    Sprite sprite;
//
//    sprite.x = std::round(p.x() * PIXELS_PER_METER - (radius*PIXELS_PER_METER/2));
//    sprite.y =  std::round(-p.y() * PIXELS_PER_METER - (radius*PIXELS_PER_METER/2));
//
//    sprite.width = radius * PIXELS_PER_METER;
//    sprite.height = radius * PIXELS_PER_METER;
//
////        sprite.red = r;
////        sprite.green = g;
////        sprite.blue = b;
////        sprite.opacity = opacity;
//
//    // 1 is a circle
//    sprite.texture = 0;
//
//    drawSprite(1, sprite);
//}

//void CanvasMessenger::drawField(const Field &field) {
//    Sprite field_sprite;
//    field_sprite.width = field.length() * PIXELS_PER_METER;
//    field_sprite.height = field.width() * PIXELS_PER_METER;
//    field_sprite.x = std::round(-field.length() * PIXELS_PER_METER/2);
//    field_sprite.y = std::round(-field.width() * PIXELS_PER_METER/2);
//
//    drawSprite(0, field_sprite);
//}







Point CanvasMessenger::Sprite::getTopLeftCorner() {
    return _center + Vector(_width, _height).rotate(_orientation);
}


std::vector<uint8_t> CanvasMessenger::Sprite::serialize() {
    std::vector<uint8_t> payload;

    Point top_left_corner = getTopLeftCorner();

    // These are all 16 bits, we need to split them
    Int16OrTwoInt8 x;
    x.base = top_left_corner.x();
    payload.insert(payload.end(), x.result[1]);
    payload.insert(payload.end(), x.result[0]);

    Int16OrTwoInt8 y;
    y.base = top_left_corner.y();
    payload.insert(payload.end(), y.result[1]);
    payload.insert(payload.end(), y.result[0]);

    Int16OrTwoInt8 width;
    width.base = _width;
    payload.insert(payload.end(), width.result[1]);
    payload.insert(payload.end(), width.result[0]);

    Int16OrTwoInt8 height;
    height.base = _height;
    payload.insert(payload.end(), height.result[1]);
    payload.insert(payload.end(), height.result[0]);

    Int16OrTwoInt8 orientation;
    orientation.base = _orientation.toDegrees();
    payload.insert(payload.end(), orientation.result[1]);
    payload.insert(payload.end(), orientation.result[0]);

    payload.insert(payload.end(), _color.a);
    payload.insert(payload.end(), _color.r);
    payload.insert(payload.end(), _color.g);
    payload.insert(payload.end(), _color.b);

    return payload;
}
