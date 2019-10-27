#pragma once

#include <Box2D/Box2D.h>

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
