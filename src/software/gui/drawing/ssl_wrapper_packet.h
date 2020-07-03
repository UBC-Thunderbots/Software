#pragma once

#include <QtWidgets/QGraphicsScene>

#include "software/gui/drawing/draw_functions.h"
#include "software/proto/messages_robocup_ssl_wrapper.pb.h"

/**
 * This file contains all the functions that allow us to draw the contents
 * of an SSL_WrapperPacket in a QGraphicsScene in Qt
 */

/**
 * Draws the contents of the SSL_WrapperPacket on the given scene.
 *
 * @param scene The scene to draw on
 * @param ssl_wrapper_packet The packet who's contents should be drawn
 */
void drawSSLWrapperPacket(QGraphicsScene* scene,
                          const SSL_WrapperPacket& ssl_wrapper_packet);

/**
 * Returns a function that represents how to draw the provided SSL WrapperPacket.
 * Consumers may call this returned function to draw the provided wrapper
 * packet onto a QGraphicsScene.
 *
 * @param ssl_wrapper_packet The wrapper packet create a DrawFunctionWrapper for
 *
 * @return A function that represents how to draw the provided world.
 */
WorldDrawFunction getDrawSSLWrapperPacketFunction(
    const SSL_WrapperPacket& ssl_wrapper_packet);
