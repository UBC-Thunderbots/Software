#pragma once

#include <QtWidgets/QGraphicsScene>

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
void drawSSLWrapperPacket(QGraphicsScene* scene, const SSL_WrapperPacket& ssl_wrapper_packet);
