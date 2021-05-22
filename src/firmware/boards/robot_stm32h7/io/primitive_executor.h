#pragma once
#include "firmware/app/primitives/primitive_manager.h"
#include "firmware/app/world/firmware_robot.h"

/**
 * Initialize the primitive executor a pointer to the world and
 * the primitive manager
 *
 * @param world A pointer to the world
 * @param primitive_manager A pointer to the primitive manager
 *
 */
void io_primitive_executor_init(FirmwareWorld_t* world,
                                PrimitiveManager_t* primitive_manager);

/*
 * Receives the primitive over the network and starts
 * the primitive.
 *
 * @param argument Unused
 */
void io_primitive_executor_task(void* argument);

/*
 * Ticks the primitive manager as fast as it will go.
 *
 * @param argument Unused
 */
void io_primitive_starter_task(void* argument);
