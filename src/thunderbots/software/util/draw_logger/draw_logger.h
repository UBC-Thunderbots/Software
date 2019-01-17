#pragma once

#include <memory>
#include <queue>

#include "thunderbots_msgs/Shape.h"

namespace Util
{
	// Singleton draw logger class (name could be changed, I don't really like it)
	class DrawLogger
	{

		using ShapeMsg = thunderbots_msgs::Shape;
		using ShapeMsgQueue = std::queue<ShapeMsg>;

	public:
		static DrawLogger& getInstance()
		{
			// Lazy instantiation
			if (!m_instance)
			{
				m_instance = std::make_shared<DrawLogger>();
			}

			return m_instance;
		}

		// TODO: add get method to get a template message
		// which gets a type param but returns a pre-gen
		// shape message, the client (ai) will populate the rest

		// void queueMessage()

	private:
		// Private constructor
		explicit DrawLogger()
		: m_shapeMsgs()
		{
		}

		// Function to verify the requested drawing command
		// is valid
		bool verifyMessage(ShapeMsg& msg)
		{
			// Verify message data
			const bool data_valid = this->verifyMessageData(msg);

			// Verify colors
			const int fill_length = msg.fill.length();
			const int stroke_length = msg.stroke.length();
			const bool fill_valid = fill_length == 3 || fill_length == 4;
			const bool stroke_valid = stroke_length == 3 || stroke_length == 4;

			return data_valid && fill_valid && stroke_valid;
		}

		// Loosely checks that the data array is in the correct form
		// Returns false if ill-formed
		//
		// Data associated with the shape; expect different form depending on type:
		// If "circle": [center_x, center_y, radius]
		// If "rect":   [x, y, width, height]
		// If "poly":   [x1, y1, x2, y2, ...]
		// If "arc":    [center_x, center_y, radius, theta_start, theta_end]
		// If "line":   [x1, y1, x2, y2]
		bool verifyMessageData(ShapeMsg& msg)
		{
			// msg.data is an array of float64 (vector of doubles)
			const int data_length = msg.data.length();
			if (msg.type == "circle")
			{
				return data_length == 3;
			}
			else if (msg.type == "rect")
			{
				return data_length == 4;
			}
			else if (msg.type == "poly")
			{
				return data_length >= 6 && data_length % 2 == 0;
			}
			else if (msg.type == "arc")
			{
				return data_length == 5;
			}
			else if (msg.type == "line")
			{
				return data_length == 4;
			}
		}


	private:
		// Private enums
		enum ShapeMsg_type
		{
			ShapeMsg_type_circle,
			ShapeMsg_type_rect,
			ShapeMsg_type_poly,
			ShapeMsg_type_arc,
			ShapeMsg_type_line
		};

		// Private members
		static std::shared_ptr<DrawLogger> m_instance;
		ShapeMsgQueue m_shapeMsgs;

	};
} // namespace Util