#include "backend/backend.h"

Backend::Backend(WorldBufferPtr world_buffer, PrimitiveVecBufferPtr primitive_vector_buffer) :
world_buffer(world_buffer), primitive_vector_buffer(primitive_vector_buffer) {}

void Backend::addWorldToBuffer(Backend::World world) {
    world_buffer->push(world);
}
Backend::PrimitiveVec Backend::getPrimitiveVecFromBuffer(){
    return primitive_vector_buffer->pullMostRecentlyAddedValue();
}


