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
        // Check if publisher is initialized before publishing messages
        // TODO: #312: refresh rate limit by comparing time delta
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

    // TODO: Point2D switch to point class
    void VisualizerMessenger::drawPoly(const std::string& layer,
                                       std::vector<Point>& vertices, DrawStyle draw_style,
                                       DrawTransform draw_transform)
    {
        ShapeMsg new_shape;
        new_shape.type = "poly";
        new_shape.data.clear();

        for (auto vertexIter = vertices.begin(); vertexIter != vertices.end();
             vertexIter++)
        {
            new_shape.data.push_back((*vertexIter).x());
            new_shape.data.push_back((*vertexIter).y());
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

    void VisualizerMessenger::drawTestLayer()
    {
        const auto vm = VisualizerMessenger::getInstance();

        auto drawAllShapes = [vm](double x, double y, DrawStyle style,
                                  DrawTransform transform) {
            // Circle test
            vm->drawEllipse("test", x + 40, y + 40, 40, 40, style, transform);

            // Ellipse test
            vm->drawEllipse("test", x + 200, y + 40, 40, 20, style, transform);

            // Square test
            vm->drawRect("test", x + 160, 20, 40, 40, style, transform);

            // Rectangle test
            vm->drawRect("test", x + 220, y + 20, 80, 20, style, transform);

            // Arc test (0 to theta < pi)
            vm->drawArc("test", x + 340, y + 40, 40, 0, 1.5, style, transform);

            // Arc test (0 to theta > pi)
            vm->drawArc("test", x + 400, y + 40, 40, 0, 4.5, style, transform);

            // Arc test (0 to theta > 2 pi)
            vm->drawArc("test", x + 460, y + 40, 40, 0, 7, style, transform);

            // Arc test (0 to negative)
            vm->drawArc("test", x + 520, y + 40, 40, 0, -0.5, style, transform);

            // Arc test (negative to 0)
            vm->drawArc("test", x + 580, y + 40, 40, -1.5, 0, style, transform);

            // Arc test (negative to > pi)
            vm->drawArc("test", x + 640, y + 40, 40, -4, 5, style, transform);

            // Line test
            vm->drawLine("test", x + 660, y + 20, 700, 40, style, transform);

            // Polygon test
            std::vector<Point> test_polygon = {
                Point(x + 720, y + 20), Point(x + 725, y + 40), Point(x + 720, y + 40),
                Point(x + 750, y + 50), Point(x + 770, y + 30), Point(x + 730, y + 20),
                Point(x + 740, y + 30)};
            vm->drawPoly("test", test_polygon, style, transform);
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

        // call the different things
        drawAllShapes(0, 0, standard_style, standard_trans);
        drawAllShapes(0, 100, standard_style, enlarged_trans);
        drawAllShapes(0, 200, standard_style, shrunk_trans);
        drawAllShapes(0, 300, standard_style, rotated_trans);

        drawAllShapes(0, 400, no_stroke_style, standard_trans);
        drawAllShapes(0, 500, no_stroke_style, enlarged_trans);
        drawAllShapes(0, 600, no_stroke_style, shrunk_trans);
        drawAllShapes(0, 700, no_stroke_style, rotated_trans);

        drawAllShapes(0, 800, patriot_style, standard_trans);
        drawAllShapes(0, 900, patriot_style, enlarged_trans);
        drawAllShapes(0, 1000, patriot_style, shrunk_trans);
        drawAllShapes(0, 1100, patriot_style, rotated_trans);
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
