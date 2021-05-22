#pragma once

/**
 *
 * Initialize the primitive executor a pointer to the world and
 * the primitive manager
 *
 * @param world A pointer to the world
 * @param primitive_manager A pointer to the primitive manager
 *
 */
void io_primitive_exector_init(FirmwareWorld_t* world,
                               PrimitiveManager_t* primitive_manager);

/*
 * Ticks the primitive manager as fast as it will go.
 */
void io_primitive_exector_task(void* argument);
