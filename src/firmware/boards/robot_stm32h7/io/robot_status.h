#pragma once

/**
 * The robot status task is responsible for querying all the sensors
 * and peripherals for their most up-to-date readings and packaging
 * them in a RobotStatus proto. It then notifies the networking
 * task to send out the updated proto.
 *
 * @param argument The robot status sender communication profile.
 *                 This profile should be created using
 *                 io_proto_multicast_communication_profile_create
 *                 in the proto_multicast library.
 */
void io_robot_status_task(void *argument);
