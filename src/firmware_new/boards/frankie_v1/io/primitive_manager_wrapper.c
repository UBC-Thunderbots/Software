#include "firmware_new/boards/frankie_v1/io/primitive_manager_wrapper.h"

#include <stdbool.h>
#include <assert.h>

static PrimitiveManager_t* primitive_manager;

static bool initialized = false;

void io_primitive_manager_wrapper_init(void){
    primitive_manager = app_primitive_manager_create();

    // TODO: do we need to do more here?

    initialized = true;
}

PrimitiveManager_t* io_primitive_manager_wrapper_getPrimitiveManager(void){
    assert(initialized);

    return primitive_manager;
}
