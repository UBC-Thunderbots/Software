#pragma once

#include <QtWidgets/QGraphicsScene>

#include "proto/ssl_vision_wrapper.pb.h"
#include "shared/robot_constants.h"
#include "software/gui/drawing/draw_functions.h"

/**
 * This file contains all the functions that allow us to draw the contents
 * of an SSLProto::SSL_WrapperPacket in a QGraphicsScene in Qt
 */

/**
 * Draws the contents of the SSLProto::SSL_WrapperPacket on the given scene.
 *
 * @param scene The scene to draw on
 * @param ssl_wrapper_packet The packet who's contents should be drawn
 * @param robot_constants The robot constants
 */
void drawSSLWrapperPacket(QGraphicsScene* scene,
                          const SSLProto::SSL_WrapperPacket& ssl_wrapper_packet,
                          const RobotConstants_t& robot_constants);

/**
 * Returns a function that represents how to draw the provided SSL WrapperPacket.
 * Consumers may call this returned function to draw the provided wrapper
 * packet onto a QGraphicsScene.
 *
 * @param ssl_wrapper_packet The wrapper packet create a DrawFunctionWrapper for
 * @param robot_constants The robot constants
 *
 * @return A function that represents how to draw the provided world.
 */
WorldDrawFunction getDrawSSLWrapperPacketFunction(
    const SSLProto::SSL_WrapperPacket& ssl_wrapper_packet,
    const RobotConstants_t& robot_constants);
