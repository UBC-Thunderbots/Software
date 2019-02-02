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

    void VisualizerMessenger::testShapes()
    {
        const auto vm = VisualizerMessenger::getInstance();

        // Helper function
        auto drawAllShapes = [vm](DrawStyle style, DrawTransform transform,
                                  double offset = 100) {
            // Circle test
            vm->drawEllipse("test", 100, offset, 50, 50, style, transform);

            // Ellipse test
            vm->drawEllipse("test", 200, offset, 50, 50, style, transform);

            // Square test
            vm->drawRect("test", 300, offset, 50, 50, style, transform);

            // Rectangle test
            vm->drawRect("test", 400, offset, 20, 80, style, transform);

            // TODO: polygon test

            // Arc standard test (about half pi clockwise)
            vm->drawArc("test", 500, offset, 50, 0, 1.5, style, transform);

            // Arc test (> pi)
            vm->drawArc("test", 600, offset, 50, 0, 3, style, transform);

            // Arc test (> 2pi; should expect angle mod'd)
            vm->drawArc("test", 700, offset, 50, 0, 4, style, transform);

            // Arc test (negative starting angle, about half pi ccw)
            vm->drawArc("test", 800, offset, 50, -1.5, 0, style, transform);

            // Arc test (negative starting angle and > 2 pi)
            vm->drawArc("test", 900, offset, 50, -4, 5, style, transform);

            // Line test
            vm->drawLine("test", 1000, offset, 150, 250, style, transform);
        };

        // Try different styles and transform
        DrawStyle standard_style = DrawStyle();

        DrawStyle no_stroke_style     = DrawStyle();
        no_stroke_style.stroke_weight = 0;

        DrawStyle patriot_style     = DrawStyle();
        patriot_style.fill          = "blue";
        patriot_style.stroke        = "red";
        patriot_style.stroke_weight = 5;

        DrawTransform standard_trans = DrawTransform();

        DrawTransform enlarged_trans = DrawTransform();
        enlarged_trans.scale         = 1.5;

        DrawTransform shrunk_trans = DrawTransform();
        shrunk_trans.scale         = 0.75;

        DrawTransform rotated_trans = DrawTransform();
        rotated_trans.rotation      = 0.52; /* ~60 deg */

        drawAllShapes(standard_style, standard_trans, 100);
        drawAllShapes(standard_style, enlarged_trans, 200);
        drawAllShapes(standard_style, shrunk_trans, 300);
        drawAllShapes(standard_style, rotated_trans, 400);

        drawAllShapes(no_stroke_style, standard_trans, 500);
        drawAllShapes(no_stroke_style, enlarged_trans, 600);
        drawAllShapes(no_stroke_style, shrunk_trans, 700);
        drawAllShapes(no_stroke_style, rotated_trans, 800);

        drawAllShapes(patriot_style, standard_trans, 900);
        drawAllShapes(patriot_style, enlarged_trans, 1000);
        drawAllShapes(patriot_style, shrunk_trans, 1100);
        drawAllShapes(patriot_style, rotated_trans, 1200);
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
    }

    void VisualizerMessenger::clearLayers()
    {
        // Clears all shape vector in all the layers
        for (std::pair<std::string, LayerMsg> layer : this->layers_name_to_msg_map)
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
