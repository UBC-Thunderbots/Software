#pragma once

#include <pb_decode.h>
#include <pb_encode.h>

#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#ifdef PLATFORMIO_BUILD
#include <proto/power_frame_msg.nanopb.h>
#else  // PLATFORMIO_BUILD
#include "proto/power_frame_msg.pb.h"
#include "proto/primitive/primitive_types.h"

extern "C"
{
#include "proto/power_frame_msg.nanopb.h"
}
#endif  // PLATFORMIO_BUILD

// The nanopb generated size isn't c++ compatible so we redefine it here
// TODO(#2592): Remove with upgrade to nanopb
#undef TbotsProto_PowerFrame_size
#define TbotsProto_PowerFrame_size                                                       \
    std::max(TbotsProto_PowerControl_size, TbotsProto_PowerStatus_size) +                \
        2 * sizeof(uint32_t) + sizeof(uint16_t)


/**
 * Serialize nanopb into its byte representation
 *
 * @param data nanopb msg to be serialized
 * @return vector of bytes representation of provided nanopb
 */
template <typename T>
std::vector<uint8_t> serializeToVector(const T& data)
{
    const pb_field_t* fields;
    int size;
    if (std::is_same<T, TbotsProto_PowerFrame>::value)
    {
        fields = TbotsProto_PowerFrame_fields;
        size   = TbotsProto_PowerFrame_size;
    }
    else if (std::is_same<T, TbotsProto_PowerControl>::value)
    {
        fields = TbotsProto_PowerControl_fields;
        size   = TbotsProto_PowerControl_size;
    }
    else if (std::is_same<T, TbotsProto_PowerStatus>::value)
    {
        fields = TbotsProto_PowerStatus_fields;
        size   = TbotsProto_PowerStatus_size;
    }
    else
    {
        throw std::runtime_error("Unexpected type as argument");
    }

    std::vector<uint8_t> buffer(size);
    pb_ostream_t stream =
        pb_ostream_from_buffer(static_cast<uint8_t*>(buffer.data()), buffer.size());
    if (!pb_encode(&stream, fields, &data))
    {
        throw std::runtime_error(
            "Failed to encode PowerFrame msg to vector when converting nanopb to vector");
    }
    return buffer;
}

/**
 * Helper function to set the type and value of a power frame's power_msg
 *
 * @param frame frame to set the power_msg of
 * @param control/status power_msg to set
 */
void setPowerMsg(TbotsProto_PowerFrame& frame, const TbotsProto_PowerControl& control)
{
    frame.which_power_msg         = TbotsProto_PowerFrame_power_control_tag;
    frame.power_msg.power_control = control;
}

void setPowerMsg(TbotsProto_PowerFrame& frame, const TbotsProto_PowerStatus& status)
{
    frame.which_power_msg        = TbotsProto_PowerFrame_power_status_tag;
    frame.power_msg.power_status = status;
}

/**
 * Creates a nanopb power status msg with provided fields
 *
 * @return a nanopb power status msg with provided fields
 */
TbotsProto_PowerStatus createNanoPbPowerStatus(float battery_voltage,
                                               float capacitor_voltage,
                                               float current_draw,
                                               TbotsProto_Geneva_Slot geneva_slot,
                                               int32_t geneva_encoder_value_a,
                                               int32_t geneva_encoder_value_b,
                                               bool breakbeam_tripped, bool flyback_fault)
{
    TbotsProto_PowerStatus status = {.battery_voltage    = battery_voltage,
                                     .capacitor_voltage  = capacitor_voltage,
                                     .current_draw       = current_draw,
                                     .geneva_slot        = geneva_slot,
                                     .geneva_encoder_value_a = geneva_encoder_value_a,
            .geneva_encoder_value_b = geneva_encoder_value_b,
                                     .breakbream_tripped = breakbeam_tripped,
                                     .flyback_fault      = flyback_fault};
    return status;
}

// These functions are only used in power service so they are excluded from platformio
// Protobuf needs additional support to run on an esp32
#ifndef PLATFORMIO_BUILD
/**
 * Converts a google protobuf power control msg to its nanopb representation
 *
 * @param google_control protobuf message to convert
 * @return a nanopb power control msg matching provided protobuf
 */
TbotsProto_PowerControl createNanoPbPowerControl(
    const TbotsProto::PowerControl& google_control)
{
    std::vector<uint8_t> serialized_proto(google_control.ByteSizeLong());
    google_control.SerializeToArray(serialized_proto.data(),
                                    static_cast<int>(google_control.ByteSizeLong()));

    TbotsProto_PowerControl nanopb_control = TbotsProto_PowerControl_init_default;

    pb_istream_t pb_in_stream = pb_istream_from_buffer(
        static_cast<uint8_t*>(serialized_proto.data()), serialized_proto.size());
    if (!pb_decode(&pb_in_stream, TbotsProto_PowerControl_fields, &nanopb_control))
    {
        throw std::runtime_error(
            "Failed to decode serialized PowerControl msg to nanopb when converting google protobuf to nanopb");
    }
    return nanopb_control;
}
/**
 * Creates a nanopb power control msg with provided fields
 * Note: Currently only used for testing
 *
 * @return a nanobp power control msg with provided fields
 */
TbotsProto_PowerControl createNanoPbPowerControl(
    ChickerCommandMode chicker_command, float kick_speed_m_per_s,
    float chip_distance_meters, AutoChipOrKickMode auto_chip_or_kick,
    float autochip_distance_meters, float autokick_speed_m_per_s,
    TbotsProto_Geneva_Slot geneva_slot)
{
    TbotsProto_PowerControl control = TbotsProto_PowerControl_init_default;
    TbotsProto_PowerControl_ChickerControl chicker =
        TbotsProto_PowerControl_ChickerControl_init_default;
    switch (chicker_command)
    {
        case ChickerCommandMode::CHIP:
            control.chicker.which_chicker_command =
                TbotsProto_PowerControl_ChickerControl_chip_distance_meters_tag;
            chicker.chicker_command.chip_distance_meters = chip_distance_meters;
            break;
        case ChickerCommandMode::KICK:
            control.chicker.which_chicker_command =
                TbotsProto_PowerControl_ChickerControl_kick_speed_m_per_s_tag;
            chicker.chicker_command.kick_speed_m_per_s = kick_speed_m_per_s;
            break;
        case ChickerCommandMode::AUTOCHIPORKICK:
            control.chicker.which_chicker_command =
                TbotsProto_PowerControl_ChickerControl_auto_chip_or_kick_tag;
            switch (auto_chip_or_kick)
            {
                case AutoChipOrKickMode::AUTOCHIP:
                    chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick =
                        TbotsProto_AutoChipOrKick_autochip_distance_meters_tag;
                    chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                        .autochip_distance_meters = autochip_distance_meters;
                    break;
                case AutoChipOrKickMode::AUTOKICK:
                    chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick =
                        TbotsProto_AutoChipOrKick_autokick_speed_m_per_s_tag;
                    chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                        .autokick_speed_m_per_s = autokick_speed_m_per_s;
                    break;
                default:
                    break;
            }
            break;
        default:
            break;
    }

    control.chicker     = chicker;
    control.geneva_slot = geneva_slot;

    return control;
}
/**
 * Converts a nanopb power status msg to its google protobuf representation
 *
 * @param status nanopb message to convert
 * @return a google protobuf power status matching provided nanopb
 */
std::unique_ptr<TbotsProto::PowerStatus> createTbotsPowerStatus(
    const TbotsProto_PowerStatus& status)
{
    auto buffer       = serializeToVector(status);
    auto proto_status = std::make_unique<TbotsProto::PowerStatus>();
    proto_status->ParseFromString(std::string(buffer.begin(), buffer.end()));
    return proto_status;
}
#endif
