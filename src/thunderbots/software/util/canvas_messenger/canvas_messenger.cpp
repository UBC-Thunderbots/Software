#include "canvas_messenger.h"

#include "util/constants.h"

namespace Util
{
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
        for (const std::pair<uint8_t, std::vector<Sprite>>& layer_pair : this->layers_map)
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
                                         const std::vector<Sprite>& sprites)
    {
        std::vector<uint8_t> payload;

        // Add the layer number at the begining of the binary message
        payload.insert(payload.end(), layer);

        // Convert each sprite into a binary message
        for (const Sprite& sprite : sprites)
        {
            payload.insert(payload.end(), sprite.texture);

            // These are all 16 bits, we need to split them
            Int16OrTwoInt8 x;
            x.base = sprite.x;
            payload.insert(payload.end(), x.result[1]);
            payload.insert(payload.end(), x.result[0]);

            Int16OrTwoInt8 y;
            y.base = sprite.y;
            payload.insert(payload.end(), y.result[1]);
            payload.insert(payload.end(), y.result[0]);

            Int16OrTwoInt8 width;
            width.base = sprite.width;
            payload.insert(payload.end(), width.result[1]);
            payload.insert(payload.end(), width.result[0]);

            Int16OrTwoInt8 height;
            height.base = sprite.height;
            payload.insert(payload.end(), height.result[1]);
            payload.insert(payload.end(), height.result[0]);

            Int16OrTwoInt8 rotation;
            rotation.base = sprite.rotation;
            payload.insert(payload.end(), rotation.result[1]);
            payload.insert(payload.end(), rotation.result[0]);

            payload.insert(payload.end(), sprite.opacity);
            payload.insert(payload.end(), sprite.red);
            payload.insert(payload.end(), sprite.green);
            payload.insert(payload.end(), sprite.blue);
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
        }
    }


    void CanvasMessenger::drawSprite(uint8_t layer, Sprite sprite)
    {
        // We simply add the sprite to the specified layer
        this->addSpriteToLayer(layer, sprite);
    }

    void CanvasMessenger::addSpriteToLayer(uint8_t layer, Sprite& sprite)
    {
        // We look if the layer exists
        if (this->layers_map.find(layer) == this->layers_map.end())
        {
            // If not, we initialize new key to empty vector pair
            this->layers_map[layer];
        }

        // and add the sprite to the layer vector
        this->layers_map[layer].emplace_back(sprite);
    }

    void CanvasMessenger::drawPoint(Point p, double radius){
        Sprite sprite;

        sprite.x = p.x() * 100;
        sprite.y =  p.y() * 100;

        sprite.width = radius * 100;
        sprite.height = radius * 100;

        sprite.red = 255;
        sprite.green = 0;
        sprite.blue = 0;

        drawSprite(1, sprite);
    }

    void CanvasMessenger::drawField(const Field &field) {
        Sprite field_sprite;
        field_sprite.width = field.length() * 100;
        field_sprite.height = field.width() * 100;

        drawSprite(0, field_sprite);
    }

}  // namespace Util
