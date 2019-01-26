#pragma once

#include <map>
#include <memory>
#include <string>

#include "thunderbots_msgs/DrawLayer.h"
#include "thunderbots_msgs/DrawShape.h"

namespace Util
{

	using LayerMsg = thunderbots_msgs::DrawLayer;
	using LayerMsgMap = std::map<std::string, LayerMsg>;
	using ShapeMsg = thunderbots_msgs::DrawShape;

	class VisualizerMessenger
	{

	public:
		typedef struct DrawStyle
		{
			// Struct constructor
			DrawStyle()
			: fill("white")
			, stroke("black")
			, stroke_weight(1)
			{ }

			std::string fill;
			std::string stroke;
			uint8_t stroke_weight;
		} DrawStyle;

		typedef struct DrawTransform
		{
			DrawTransform()
			: rotation(0.0)
			, scale(1.0)
			{ }

			double rotation;
			double scale;
		} DrawTransform;

	public:
		// Singleton getter
		static std::shared_ptr<VisualizerMessenger> getInstance()
		{
			static std::shared_ptr<VisualizerMessenger> draw_logger(new VisualizerMessenger);
			return draw_logger;
		}

		// Get map of layers
		const LayerMsgMap& getLayerMap() const;

		// Clear the content of the layers on every update
		void clearLayers();

		// TODO: Add cached drawing styles
		// void setDrawStyle(DrawStyle style);
		// void setFill(std::string color);
		// void setStroke(std::string color);
		// void setStrokeWeight(uint8_t weight);

		// TODO: Add cached drawing transformations
		// void setDrawTransforma(DrawTransform transform);
		// void setRotation(double rad);
		// void setscale(double scale);
		
		// TODO: Add cached layer
		// void setLayer(std::string& name);

		void ellipse(std::string& layer, double cx, double cy, double r1, double r2, DrawStyle draw_style = DrawStyle(), DrawTransform draw_transform = DrawTransform());
		void rect(std::string& layer, double x, double y, double w, double h, DrawStyle draw_style = DrawStyle(), DrawTransform draw_transform = DrawTransform());
        // TODO: figure out a way to deal with variable length vertices
        void poly();
		void arc(std::string& layer, double cx, double cy, double radius, double theta_start, double theta_end, DrawStyle draw_style = DrawStyle(), DrawTransform draw_transform = DrawTransform());
		void line(std::string& layer, double x1, double y1, double x2, double y2, DrawStyle draw_style = DrawStyle(), DrawTransform draw_transform = DrawTransform());

	private:
		explicit VisualizerMessenger()
		: m_draw_layers()
		{
			buildLayers();
		}

		void buildLayers();
		void applyDrawStyleToMsg(ShapeMsg& shape_msg, DrawStyle& style);
		void applyDrawTransformToMsg(ShapeMsg& shape_msg, DrawTransform& transform);
		void addShapeToLayer(std::string& layer, ShapeMsg shape);

	private:
		LayerMsgMap m_draw_layers;
	};
} // namespace Util
