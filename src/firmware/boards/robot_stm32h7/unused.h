#pragma once

/**
 * Marks a function parameter as unused.
 *
 * This macro should be wrapped around the parameter name in the function definition’s
 * parameter list.
 *
 * @param x the name of the parameter
 */
#define UNUSED(x) (void)(x)
