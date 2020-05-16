#pragma once

#include <Box2D/Box2D.h>

#include "software/new_geom/point.h"
#include "software/new_geom/vector.h"

/**
 * These functions are utilities and convenience functions to make certain operations
 * with Box2D easier
 */

/**
 * Returns true if the b2Body exists in the world, and false otherwise.
 * If either body or world are null, false is returned. This function
 * does not take ownership of the pointers and is not responsible for
 * allocating or deallocating memory.
 *
 * @param body The Box2D body to search for in the world
 * @param world The world to search for the Box2D boxy
 *
 * @return true if the body exists in the world, and false otherwise. If either the
 * body or the world are null, false is returned.
 */
bool bodyExistsInWorld(b2Body* body, b2World* world);

/**
 * Converts the given Point into a b2Vec2, a 2D vector used in Box2D.
 *
 * @param point The Point to convert to a b2Vec2
 *
 * @return A b2Vec2 equivalent to the given Point
 */
b2Vec2 createVec2(const Point& point);

/**
 * Converts the given Vector into a b2Vec2, a 2D vector used in Box2D.
 *
 * @param vector The Vector to convert to a b2Vec2
 *
 * @return A b2Vec2 equivalent to the given Vector
 */
b2Vec2 createVec2(const Vector& vector);

/**
 * Converts the given b2Vec2 into a Point
 *
 * @param vec The b2Vec2 to convert to a Point
 *
 * @return A Point equivalent to the given b2Vec2
 */
Point createPoint(const b2Vec2& vec);

/**
 * Converts the given b2Vec2 into a Vector
 *
 * @param vec The b2Vec2 to convert to a Vector
 *
 * @return A Vector equivalent to the given b2Vec2
 */
Vector createVector(const b2Vec2& vec);

/**
 * Calculates and returns the area of the given polygon. The polygon must be convex.
 *
 * @param polygon The polygon to calculate the area of
 *
 * @return the area of the polygon, in m^2
 */
float polygonArea(const b2PolygonShape& polygon);
