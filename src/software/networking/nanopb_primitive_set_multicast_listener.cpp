#include "software/networking/nanopb_primitive_set_multicast_listener.h"

#include "pb_decode.h"

extern "C" {
#include "shared/proto/tbots_software_msgs.nanopb.h"
}

#include "software/logger/logger.h"

NanoPbPrimitiveSetMulticastListener::NanoPbPrimitiveSetMulticastListener(
    boost::asio::io_service& io_service, const std::string& ip_address,
    const unsigned short port, std::function<void(std::map<RobotId, PrimitiveMsg>&)> receive_callback)
    : multicast_listener(
          io_service, ip_address, port,
          boost::bind(&NanoPbPrimitiveSetMulticastListener::handleDataReception, this,
                      _1)),
                      receive_callback(receive_callback)
{
}

void NanoPbPrimitiveSetMulticastListener::handleDataReception(std::vector<uint8_t>& data)
{
    std::map<RobotId, PrimitiveMsg> primitive_map;

    auto handle_field_callback = [](pb_istream_t *istream, const pb_field_t *field, void **arg){
        auto map = static_cast<std::map<RobotId, PrimitiveMsg>*>(*arg);
        if (field->tag == PrimitiveSetMsg_robot_primitives_tag){
            PrimitiveSetMsg_RobotPrimitivesEntry primitive_map_entry;
            if (!pb_decode(istream, PrimitiveSetMsg_RobotPrimitivesEntry_fields, &primitive_map_entry)){
                // TODO: log or something here
                // TODO: what the heck does this return value mean?
                return false;
            }
            (*map)[primitive_map_entry.key] = primitive_map_entry.value;
//            map->insert(primitive_map_entry.key, primitive_map_entry.value);
        }
        // TODO: what the heck does this return value mean?
        return true;
    };

    PrimitiveSetMsg primitive_set_msg;
    primitive_set_msg.robot_primitives.funcs.decode = handle_field_callback;
    primitive_set_msg.robot_primitives.arg = static_cast<void*>(&primitive_map);
    pb_istream_t pb_in_stream = pb_istream_from_buffer(
        static_cast<uint8_t*>(data.data()), data.size());
    const bool parsing_succeeded =
        pb_decode(&pb_in_stream, PrimitiveSetMsg_fields, &primitive_set_msg);
    if (!parsing_succeeded)
    {
        LOG(WARNING) << "Failed to parse received packet into a NanoPb PrimitiveSetMsg";
        return;
    }
    receive_callback(primitive_map);
}
