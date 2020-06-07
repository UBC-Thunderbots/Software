#include "firmware_new/boards/frankie_v1/io/ai_communicator.h"

#include <stdlib.h>

#include "pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

typedef struct AICommunicator
{
    // connection data
    ip_addr_t* multicast_address;
    unsigned vision_port;
    unsigned primitive_port;
    unsigned robot_status_port;

    // connections
    struct netconn* primitive_multicast_conn;
    struct netconn* vision_multicast_conn;
    struct netconn* robot_status_multicast_conn;

    // callbacks
    primitive_callback_t primitive_callback;
    vision_callback_t vision_callback;

    // buffers
    uint8_t protobuf_send_buffer[MAXIMUM_TRANSFER_UNIT_BYTES];
    struct netbuf* tx_buf;
    struct netbuf* rx_buf;

} AICommunicator_t;

AICommunicator_t* io_ai_communicator_create(const char* multicast_address,
                                            unsigned vision_port, unsigned primtive_port,
                                            unsigned robot_status_port,
                                            vision_callback_t vision_callback,
                                            primitive_callback_t primitive_callback)
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

    // store
    ai_communicator->multicast_address           = multicast_address;
    ai_communicator->primitive_multicast_conn    = primitive_multicast_conn;
    ai_communicator->vision_multicast_conn       = vision_multicast_conn;
    ai_communicator->robot_status_multicast_conn = robot_status_multicast_conn;
    ai_communicator->primitive_callback          = primitive_callback;
    ai_communicator->vision_callback             = vision_callback;
    ai_communicator->vision_port                 = vision_port;
    ai_communicator->primitive_port              = primtive_port;
    ai_communicator->robot_status_port           = robot_status_port;

    return ai_communicator;
}

void io_ai_communicator_destroy(AICommunicator_t* io_ai_communicator)
{
    // leave multicast groups
    netconn_join_leave_group(io_ai_communicator->primitive_multicast_conn,
                             multicast_address, NULL, NETCONN_LEAVE);
    netconn_join_leave_group(io_ai_communicator->vision_multicast_conn, multicast_address,
                             NULL, NETCONN_LEAVE);
    netconn_join_leave_group(io_ai_communicator->robot_status_multicast_conn,
                             multicast_address, NULL, NETCONN_LEAVE);

    // delete all netconns
    netconn_delete(io_ai_communicator->primitive_multicast_conn);
    netconn_delete(io_ai_communicator->vision_multicast_conn);
    netconn_delete(io_ai_communicator->robot_status_multicast_conn);

    free(io_ai_communicator->multicast_address);
    free(io_ai_communicator);
}

void io_ai_communicator_sendTbotsRobotMsg(AICommunicator_t* io_ai_communicator,
                                         TbotsRobotMsg& robot_msg)
{
    tx_buf = netbuf_new();

    // TODO make sure sizeof(status) works
    netbuf_alloc(tx_buf, sizeof(robot_msg));

    // serialize proto
    pb_ostream_t stream = pb_ostream_from_buffer(protobuf_send_buffer, MAXIMUM_TRANSFER_UNIT_BYTES);
    pb_encode(&stream, TbotsRobotMsg_fields, &ack);

    tx_buf->p->payload = buffer;

    // send robot status
    netconn_sendto(io_ai_communicator->robot_status_port, io_ai_communicator->tx_buf,
                   io_ai_communicator->multicast_address io_ai_communicator->send_port);

    netbuf_delete(tx_buf);
}
