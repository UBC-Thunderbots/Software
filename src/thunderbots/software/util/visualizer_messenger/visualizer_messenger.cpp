#include "visualizer_messenger.h"

#include "util/constants.h"
#include "util/logger/init.h"

namespace Util
{
    std::shared_ptr<VisualizerMessenger> VisualizerMessenger::getInstance()
    {
        static std::shared_ptr<VisualizerMessenger> vm(new VisualizerMessenger);
        return vm;
    }

    void VisualizerMessenger::initializePublisher(ros::NodeHandle node_handle)
    {
        if (this->publisher)
        {
            LOG(WARNING) << "Re-initializing visualizer messenger singleton publisher";
        }

        this->publisher = node_handle.advertise<LayerMsg>(
            Util::Constants::VISUALIZER_DRAW_LAYER_TOPIC, 8);
    }

    const LayerMsgMap& VisualizerMessenger::getLayerMap() const
    {
        return this->layers_name_to_msg_map;
    }

    void VisualizerMessenger::publishAndClearLayers()
    {
        // Limit rate of the message publishing
        // Get the time right now
        const time_point now     = std::chrono::system_clock::now();
        const int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                       now - time_last_published)
                                       .count();
        const double elapsed_ms = elapsed_ns / 1.0e6;

        // Do not do anything if the time passed hasn't been
        // long enough
        if (elapsed_ms < DESIRED_PERIOD_MS)
            return;

        // Check if publisher is initialized before publishing messages
        if (!this->publisher)
        {
            LOG(WARNING)
                << "Publishing requested when visualizer messenger publisher is not initialized";
        }
        else
        {
            // Send layer messages
            for (const std::pair<std::string, LayerMsg>& layer_msg_pair : getLayerMap())
            {
                if (!layer_msg_pair.second.shapes.empty())
                {
                    this->publisher.publish(layer_msg_pair.second);
                }
            }
        }

        // Clear shapes in layers of current frame/tick
        clearLayers();

        // Update last published time
        time_last_published = now;
    }

    void VisualizerMessenger::clearLayers()
    {
        // Clears all shape vector in all the layers
        for (auto& layer : this->layers_name_to_msg_map)
        {
            layer.second.shapes.clear();
        }
    }

    void VisualizerMessenger::drawEllipse(const std::string& layer, double cx, double cy,
                                          double r1, double r2, DrawStyle draw_style,
                                          DrawTransform draw_transform)
    {
        ShapeMsg new_shape;
        new_shape.type = "ellipse";
        new_shape.data.clear();
        new_shape.data.push_back(cx);
        new_shape.data.push_back(cy);
        new_shape.data.push_back(r1);
        new_shape.data.push_back(r2);

        applyDrawStyleToMsg(new_shape, draw_style);
        applyDrawTransformToMsg(new_shape, draw_transform);
        addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::drawRect(const std::string& layer, double x, double y,
                                       double w, double h, DrawStyle draw_style,
                                       DrawTransform draw_transform)
    {
        ShapeMsg new_shape;
        new_shape.type = "rect";
        new_shape.data.clear();
        new_shape.data.push_back(x);
        new_shape.data.push_back(y);
        new_shape.data.push_back(w);
        new_shape.data.push_back(h);

        applyDrawStyleToMsg(new_shape, draw_style);
        applyDrawTransformToMsg(new_shape, draw_transform);
        addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::drawPoly(const std::string& layer,
                                       std::vector<Point2D>& vertices,
                                       DrawStyle draw_style, DrawTransform draw_transform)
    {
        ShapeMsg new_shape;
        new_shape.type = "poly";
        new_shape.data.clear();

        for (auto vertexIter = vertices.begin(); vertexIter != vertices.end();
             vertexIter++)
        {
            new_shape.data.push_back((*vertexIter).x);
            new_shape.data.push_back((*vertexIter).y);
        }

        applyDrawStyleToMsg(new_shape, draw_style);
        applyDrawTransformToMsg(new_shape, draw_transform);
        addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::drawArc(const std::string& layer, double cx, double cy,
                                      double radius, double theta_start, double theta_end,
                                      DrawStyle draw_style, DrawTransform draw_transform)
    {
        ShapeMsg new_shape;
        new_shape.type = "arc";
        new_shape.data.clear();
        new_shape.data.push_back(cx);
        new_shape.data.push_back(cy);
        new_shape.data.push_back(radius);
        new_shape.data.push_back(theta_start);
        new_shape.data.push_back(theta_end);

        applyDrawStyleToMsg(new_shape, draw_style);
        applyDrawTransformToMsg(new_shape, draw_transform);
        addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::drawLine(const std::string& layer, double x1, double y1,
                                       double x2, double y2, DrawStyle draw_style,
                                       DrawTransform draw_transform)
    {
        ShapeMsg new_shape;
        new_shape.type = "line";
        new_shape.data.clear();
        new_shape.data.push_back(x1);
        new_shape.data.push_back(y1);
        new_shape.data.push_back(x2);
        new_shape.data.push_back(y2);

        applyDrawStyleToMsg(new_shape, draw_style);
        applyDrawTransformToMsg(new_shape, draw_transform);
        addShapeToLayer(layer, new_shape);
    }

    void VisualizerMessenger::buildLayers()
    {
        // TODO: #268 list these existing layer names elsewhere where it's more obvious
        std::vector<std::string> layer_names = std::vector<std::string>(
            {"field", "ball", "ball_vel", "friendly_robot", "friendly_robot_vel",
             "enemy_robot", "enemy_robot_vel", "nav", "rule", "passing", "test", "misc"});

        for (std::string layer_name : layer_names)
        {
            LayerMsg new_layer_msg;
            new_layer_msg.layer_name = layer_name;
            this->layers_name_to_msg_map.insert(
                std::pair<std::string, LayerMsg>(layer_name, new_layer_msg));
        }
    }

    void VisualizerMessenger::applyDrawStyleToMsg(ShapeMsg& shape_msg, DrawStyle& style)
    {
        shape_msg.fill          = style.fill;
        shape_msg.stroke        = style.stroke;
        shape_msg.stroke_weight = style.stroke_weight;
    }

    void VisualizerMessenger::applyDrawTransformToMsg(ShapeMsg& shape_msg,
                                                      DrawTransform& transform)
    {
        shape_msg.transform_rotation = transform.rotation;
        shape_msg.transform_scale    = transform.scale;
    }

    void VisualizerMessenger::addShapeToLayer(const std::string& layer, ShapeMsg& shape)
    {
        if (this->layers_name_to_msg_map.find(layer) !=
            this->layers_name_to_msg_map.end())
        {
            this->layers_name_to_msg_map[layer].shapes.emplace_back(shape);
        }
        else
        {
            LOG(WARNING) << "Referenced layer (" << layer << ") is undefined\n";
        }
    }

}  // namespace Util
