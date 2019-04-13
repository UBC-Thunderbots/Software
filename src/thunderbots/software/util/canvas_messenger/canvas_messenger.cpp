#include "canvas_messenger.h"

#include "util/constants.h"

namespace Util
{
    std::shared_ptr<CanvasMessenger> CanvasMessenger::getInstance()
    {
        static std::shared_ptr<CanvasMessenger> vm(new CanvasMessenger);
        return vm;
    }

    void CanvasMessenger::initializePublisher(ros::NodeHandle node_handle)
    {
        this->publisher = node_handle.advertise<CanvasLayer>(
            Util::Constants::VISUALIZER_DRAW_LAYER_TOPIC, 8);
    }

    void CanvasMessenger::publishAndClearLayers()
    {
        // Send a payload per layer of messages
        for (const std::pair<uint8_t, SpriteVector>& layer_pair : this->layers_map)
        {
            // First is the layer number
            const uint8_t layer_number = layer_pair.first;

            // Second is the vector that contains the sprites
            const SpriteVector& sprites = layer_pair.second;

            // Only send the non-empty layers
            if (!layer_pair.second.empty())
            {
                this->publishPayload(layer_number, sprites);
            }
        }

        // Clear shapes in layers of current frame/tick
        this->clearLayers();
    }

    void CanvasMessenger::publishPayload(uint8_t layer, const SpriteVector& sprites)
    {
        std::vector<uint8_t> payload;
        payload.insert(payload.end(), layer);

        for (const Sprite& sprite : sprites)
        {
            // Packing texture (8 bit) and flag (8 bit) data into a single 16 bit int
            payload.insert(payload.end(), sprite.texture);

            // These are all 16 bits
            int16to8 x;
            x.base = sprite.x;
            payload.insert(payload.end(), x.result[1]);
            payload.insert(payload.end(), x.result[0]);

            int16to8 y;
            y.base = sprite.y;
            payload.insert(payload.end(), y.result[1]);
            payload.insert(payload.end(), y.result[0]);

            int16to8 width;
            width.base = sprite.width;
            payload.insert(payload.end(), width.result[1]);
            payload.insert(payload.end(), width.result[0]);

            int16to8 height;
            height.base = sprite.height;
            payload.insert(payload.end(), height.result[1]);
            payload.insert(payload.end(), height.result[0]);

            int16to8 rotation;
            rotation.base = sprite.rotation;
            payload.insert(payload.end(), rotation.result[1]);
            payload.insert(payload.end(), rotation.result[0]);

            payload.insert(payload.end(), sprite.opacity);
            payload.insert(payload.end(), sprite.red);
            payload.insert(payload.end(), sprite.green);
            payload.insert(payload.end(), sprite.blue);
        }

        CanvasLayer new_layer;
        new_layer.data = payload;

        this->publisher.publish(new_layer);
    }

    void CanvasMessenger::clearLayers()
    {
        for (auto& layer : this->layers_map)
        {
            layer.second.clear();
        }
    }

    void CanvasMessenger::drawSprite(uint8_t layer, Sprite sprite)
    {
        this->addSpriteToLayer(layer, sprite);
    }

    void CanvasMessenger::addSpriteToLayer(uint8_t layer, Sprite& sprite)
    {
        if (this->layers_map.find(layer) == this->layers_map.end())
        {
            // Initialize new key to empty vector pair
            this->layers_map[layer];
        }

        this->layers_map[layer].emplace_back(sprite);
    }

}  // namespace Util