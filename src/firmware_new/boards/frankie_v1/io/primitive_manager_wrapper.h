#pragma once

// TODO: decide if there's a more appropriate location for this file

// TODO: better name for this file/"class"?

#include "firmware/app/primitives/primitive_manager.h"

// TODO: do locking in this wrapper?

// TODO: jdoc here
void io_primitive_manager_wrapper_init(void);

// TODO: jdoc here
PrimitiveManager_t* io_primitive_manager_wrapper_getPrimitiveManager(void);
