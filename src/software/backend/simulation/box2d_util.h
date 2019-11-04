#pragma once

#include <Box2D/Box2D.h>

#include "software/geom/point.h"

/**
 * These functions are utilities and convenience functions to make certain operations
 * with Box2D easier
 */

/**
 * Returns true if the b2Body exists in the world, and false otherwise.
 * If either body or world are null, false is returned
 *
 * @param body The Box2D body to search for in the world
 * @param world The world to search for the Box2D boxy
 *
 * @return true if the body exists in the world, and false otherwise. If either the
 * body or the world are null, false is returned.
 */
bool bodyExistsInWorld(b2Body* body, std::shared_ptr<b2World> world);

/**
 * Converts the given Point into a b2Vec2, a 2D vector used in Box2D.
 *
 * @param point The Point to convert to a b2Vec2
 *
 * @return A b2Vec2 equivalent to the given Point
 */
b2Vec2 createVec2(const Point& point);
