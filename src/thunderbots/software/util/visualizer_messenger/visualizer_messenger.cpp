#include "visualizer_messenger.h"

namespace Util
{

const LayerMsgMap& VisualizerMessenger::getLayerMap() const
{
	return m_draw_layers;
}

void VisualizerMessenger::clearLayers()
{
	// Clears all shape vector in all the layers
	for (std::pair<std::string, LayerMsg> layer : m_draw_layers)
	{
		layer.second.shapes.clear();
	}
}

void VisualizerMessenger::ellipse(
	const std::string& layer,
	double cx, double cy, double r1, double r2,
	DrawStyle draw_style,
	DrawTransform draw_transform
)
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

void VisualizerMessenger::rect(
	const std::string& layer,
	double x, double y, double w, double h,
	DrawStyle draw_style,
	DrawTransform draw_transform
)
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

void VisualizerMessenger::arc(
	const std::string& layer,
	double cx, double cy, double radius, double theta_start, double theta_end,
	DrawStyle draw_style,
	DrawTransform draw_transform
)
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

void VisualizerMessenger::line(
	const std::string& layer,
	double x1, double y1, double x2, double y2,
	DrawStyle draw_style,
	DrawTransform draw_transform
)
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
	// TODO: #259 list these existing layer names elsewhere where it's more obvious
	std::vector<std::string> layer_names = std::vector<std::string>(
		{
			"field",
			"ball",
			"ball_vel",
			"fiendly_robot",
			"fiendly_robot_vel",
			"enemy_robot",
			"enemy_robot_vel",
			"nav",
			"rule",
			"passing",
			"test",
			"misc"
		}
	);

	for (std::string layer_name : layer_names)
	{
		LayerMsg new_layer_msg;
		new_layer_msg.name = layer_name;
		m_draw_layers.insert(std::pair<std::string, LayerMsg>(layer_name, new_layer_msg));
	}
}

void VisualizerMessenger::applyDrawStyleToMsg(ShapeMsg& shape_msg, DrawStyle& style)
{
    shape_msg.fill          = style.fill;
    shape_msg.stroke        = style.stroke;
    shape_msg.stroke_weight = style.stroke_weight;
}

void VisualizerMessenger::applyDrawTransformToMsg(ShapeMsg& shape_msg, DrawTransform& transform)
{
    shape_msg.transform_rotation = transform.rotation;
    shape_msg.transform_scale    = transform.scale;
}

void VisualizerMessenger::addShapeToLayer(const std::string& layer, ShapeMsg& shape)
{
	if (m_draw_layers.find(layer) != m_draw_layers.end())
	{
		m_draw_layers[layer].shapes.emplace_back(shape);
	}
}

} // namespace Util