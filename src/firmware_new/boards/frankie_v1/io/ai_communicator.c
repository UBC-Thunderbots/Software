#include "firmware_new/boards/frankie_v1/io/ai_communicator.h"

#include <stdlib.h>

typedef struct AICommunicator
{
    ip_addr_t* multicast_address;
    struct netconn* primitive_multicast_conn;
    struct netconn* vision_multicast_conn;
    struct netconn* robot_status_multicast_conn;
} AICommunicator_t;

AICommunicator_t* io_ai_communicator_create(const char* multicast_address,
                                            unsigned vision_port, unsigned primtive_port,
                                            unsigned robot_status_port)
{
    AICommunicator_t* ai_communicator =
        (AICommunicator_t*)malloc(sizeof(AICommunicator_t));

    // store the multicast address in the correct format
    ip_addr_t* multicast_address_ptr = (ip_addr_t*)malloc(sizeof(ip_addr_t));
    ip6addr_aton(multicast_address, multicast_address_ptr);

    // create multicast connections
    struct netconn* primitive_multicast_conn    = netconn_new(NETCONN_UDP_IPV6);
    struct netconn* vision_multicast_conn       = netconn_new(NETCONN_UDP_IPV6);
    struct netconn* robot_status_multicast_conn = netconn_new(NETCONN_UDP_IPV6);

    // Bind the socket to the multicast address and port we then use that
    // connection profile to join the specified multicast group.
    netconn_bind(primitive_multicast_conn, multicast_address_ptr, primtive_port);
    netconn_bind(vision_multicast_conn, multicast_address_ptr, vision_port);
    netconn_bind(robot_status_multicast_conn, multicast_address_ptr, robot_status_port);

    // join multicast groups
    netconn_join_leave_group(primitive_multicast_conn, multicast_address, NULL,
                             NETCONN_JOIN);
    netconn_join_leave_group(vision_multicast_conn, multicast_address, NULL,
                             NETCONN_JOIN);
    netconn_join_leave_group(robot_status_multicast_conn, multicast_address, NULL,
                             NETCONN_JOIN);

    // store connections
    ai_communicator->multicast_address = multicast_address;
    ai_communicator->primitive_multicast_conn = primitive_multicast_conn;
    ai_communicator->vision_multicast_conn = vision_multicast_conn;
    ai_communicator->robot_status_multicast_conn = robot_status_multicast_conn;

    return ai_communicator;
}

void io_ai_communicator_destroy(AICommunicator_t* io_ai_communicator)
{
    // leave multicast groups
    netconn_join_leave_group(io_ai_communicator->primitive_multicast_conn, multicast_address, NULL,
                             NETCONN_LEAVE);
    netconn_join_leave_group(io_ai_communicator->vision_multicast_conn, multicast_address, NULL,
                             NETCONN_LEAVE);
    netconn_join_leave_group(io_ai_communicator->robot_status_multicast_conn, multicast_address, NULL,
                             NETCONN_LEAVE);

    // delete all netconns
    netconn_delete(io_ai_communicator->primitive_multicast_conn);
    netconn_delete(io_ai_communicator->vision_multicast_conn);
    netconn_delete(io_ai_communicator->robot_status_multicast_conn);
    
    free(io_ai_communicator->multicast_address);
    free(io_ai_communicator);
}

void io_ai_communicator_sendRobotStatus()
{
    switch (direction)
    {
        case COUNTERCLOCKWISE:
            io_gpio_pin_setActive(motor_driver->direction_pin);
            return;
        case CLOCKWISE:
            io_gpio_pin_setInactive(motor_driver->direction_pin);
            return;
    }
}

void io_ai_communicator_registerVisionCallback()
{
    io_pwm_pin_setPwm(motor_driver->pwm_pin, pwm_percentage);
}
