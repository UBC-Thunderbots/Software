#pragma once

#include <vector>

typedef enum { PRIM_MOVE_ID = 0, PRIM_SHOOT_ID = 1 } PrimtiveID;

/**
 * Defines a Robot Primitive, which is the most basic action / unit of work a robot can
 * do.
 *
 * This is an Abstract, pure-virtual class. It is meant to define the interace that all
 * Primitives must follow.
 * Other classes should inherit from this class and implement the methods to create a
 * useable Primitive class.
 */
class Primitive
{
   public:
};
