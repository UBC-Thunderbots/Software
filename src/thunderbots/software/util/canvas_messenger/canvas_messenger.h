#pragma once

#include <ros/ros.h>

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "thunderbots_msgs/CanvasLayer.h"

// Forward declaration
namespace ros
{
    class NodeHandle;
    class Publisher;
}  // namespace ros

namespace Util
{
    using CanvasLayer = thunderbots_msgs::CanvasLayer;

    class CanvasMessenger
    {
       public:
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

        using SpriteVector = std::vector<Sprite>;
        using LayerMap     = std::map<uint8_t, SpriteVector>;

       public:
        static std::shared_ptr<CanvasMessenger> getInstance();
        void initializePublisher(ros::NodeHandle node_handle);
        void publishAndClearLayers();
        void clearLayers();

        void drawSprite(uint8_t layer, Sprite sprite = Sprite());

       private:
        union int16to8 {
            int16_t base;
            uint8_t result[2];
        };

       private:
        /**
         * Constructor; initializes an empty layers map then populates it
         */
        explicit CanvasMessenger() : layers_map(), publisher() {}

        void publishPayload(uint8_t layer, const SpriteVector& shapes);
        void addSpriteToLayer(uint8_t layer, Sprite& sprite_data);

       private:
        LayerMap layers_map;
        ros::Publisher publisher;
    };
}  // namespace Util