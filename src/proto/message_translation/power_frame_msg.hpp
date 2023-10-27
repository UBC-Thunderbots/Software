#pragma once

#include <pb_decode.h>
#include <pb_encode.h>
#include <proto/power_frame_msg.nanopb.h>

#include <cmath>
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <vector>

#ifdef PLATFORMIO_BUILD
#include <proto/power_frame_msg.nanopb.h>
#else  // PLATFORMIO_BUILD
#include "proto/power_frame_msg.pb.h"
#include "proto/primitive.pb.h"
#include "proto/primitive/primitive_types.h"
#include "shared/constants.h"

extern "C"
{
#include "proto/power_frame_msg.nanopb.h"
}
#endif  // PLATFORMIO_BUILD

// The nanopb generated size isn't c++ compatible so we redefine it here
// TODO(#2592): Remove with upgrade to nanopb
#undef TbotsProto_PowerFrame_size
#define TbotsProto_PowerFrame_size                                                       \
    std::max(TbotsProto_PowerPulseControl_size, TbotsProto_PowerStatus_size) +           \
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
    else if (std::is_same<T, TbotsProto_PowerPulseControl>::value)
    {
        fields = TbotsProto_PowerPulseControl_fields;
        size   = TbotsProto_PowerPulseControl_size;
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
void inline setPowerMsg(TbotsProto_PowerFrame& frame,
                        const TbotsProto_PowerPulseControl& control)
{
    frame.which_power_msg         = TbotsProto_PowerFrame_power_control_tag;
    frame.power_msg.power_control = control;
}

void inline setPowerMsg(TbotsProto_PowerFrame& frame,
                        const TbotsProto_PowerStatus& status)
{
    frame.which_power_msg        = TbotsProto_PowerFrame_power_status_tag;
    frame.power_msg.power_status = status;
}

/**
 * Creates a nanopb power status msg with provided fields
 *
 * @return a nanopb power status msg with provided fields
 */
TbotsProto_PowerStatus inline createNanoPbPowerStatus(float battery_voltage,
                                                      float capacitor_voltage,
                                                      float current_draw,
                                                      TbotsProto_Geneva_Slot geneva_slot,
                                                      uint32_t time_since_last_msg_ms,
                                                      bool breakbeam_tripped)
{
    TbotsProto_PowerStatus status = {.battery_voltage        = battery_voltage,
                                     .capacitor_voltage      = capacitor_voltage,
                                     .current_draw           = current_draw,
                                     .geneva_slot            = geneva_slot,
                                     .time_since_last_msg_ms = time_since_last_msg_ms,
                                     .breakbeam_tripped      = breakbeam_tripped};
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

TbotsProto_PowerPulseControl inline createNanoPbPowerPulseControl(
    const TbotsProto::PowerControl& google_control, int kick_slope, int kick_constant,
    int chip_pulse_width)
{
    TbotsProto_PowerPulseControl nanopb_control =
        TbotsProto_PowerPulseControl_init_default;

    switch (google_control.chicker().chicker_command_case())
    {
        case TbotsProto::PowerControl::ChickerControl::kKickSpeedMPerS:
            nanopb_control.chicker.which_chicker_command =
                TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag;
            nanopb_control.chicker.chicker_command.kick_pulse_width =
                kick_slope *
                    static_cast<uint32_t>(google_control.chicker().kick_speed_m_per_s()) +
                kick_constant;
            break;
        case TbotsProto::PowerControl::ChickerControl::kChipDistanceMeters:
            nanopb_control.chicker.which_chicker_command =
                TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag;
            nanopb_control.chicker.chicker_command.chip_pulse_width = chip_pulse_width;
            break;
        case TbotsProto::PowerControl::ChickerControl::kAutoChipOrKick:
            nanopb_control.chicker.which_chicker_command =
                TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag;
            switch (google_control.chicker().auto_chip_or_kick().auto_chip_or_kick_case())
            {
                case TbotsProto::AutoChipOrKick::kAutokickSpeedMPerS:
                    nanopb_control.chicker.chicker_command.auto_chip_or_kick
                        .which_auto_chip_or_kick =
                        TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag;
                    nanopb_control.chicker.chicker_command.auto_chip_or_kick
                        .auto_chip_or_kick.autokick_pulse_width =
                        kick_slope *
                            static_cast<uint32_t>(google_control.chicker()
                                                      .auto_chip_or_kick()
                                                      .autokick_speed_m_per_s()) +
                        kick_constant;
                    break;
                case TbotsProto::AutoChipOrKick::kAutochipDistanceMeters:
                    nanopb_control.chicker.chicker_command.auto_chip_or_kick
                        .which_auto_chip_or_kick =
                        TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag;
                    nanopb_control.chicker.chicker_command.auto_chip_or_kick
                        .auto_chip_or_kick.autochip_pulse_width = chip_pulse_width;
                    break;

                default:
                    break;
            }
            break;
        default:
            break;
    }
    switch (google_control.geneva_slot())
    {
        case TbotsProto::Geneva::LEFT:
            nanopb_control.geneva_slot = TbotsProto_Geneva_Slot_LEFT;
            break;
        case TbotsProto::Geneva::CENTRE_LEFT:
            nanopb_control.geneva_slot = TbotsProto_Geneva_Slot_CENTRE_LEFT;
            break;
        case TbotsProto::Geneva::CENTRE:
            nanopb_control.geneva_slot = TbotsProto_Geneva_Slot_CENTRE;
            break;
        case TbotsProto::Geneva::CENTRE_RIGHT:
            nanopb_control.geneva_slot = TbotsProto_Geneva_Slot_CENTRE_RIGHT;
            break;
        case TbotsProto::Geneva::RIGHT:
            nanopb_control.geneva_slot = TbotsProto_Geneva_Slot_RIGHT;
            break;
        default:
            break;
    }
    return nanopb_control;
}
/**
 * Creates a nanopb power control msg with provided fields
 * Note: Currently only used for testing
 *
 * @return a nanobp power control msg with provided fields
 */
TbotsProto_PowerPulseControl inline createNanoPbPowerPulseControl(
    ChickerCommandMode chicker_command, uint32_t kick_pulse_width,
    uint32_t chip_pulse_width, AutoChipOrKickMode auto_chip_or_kick,
    uint32_t autochip_pulse_width, uint32_t autokick_pulse_width,
    TbotsProto_Geneva_Slot geneva_slot)
{
    TbotsProto_PowerPulseControl control = TbotsProto_PowerPulseControl_init_default;
    TbotsProto_PowerPulseControl_ChickerControl chicker =
        TbotsProto_PowerPulseControl_ChickerControl_init_default;
    switch (chicker_command)
    {
        case ChickerCommandMode::CHIP:
            control.chicker.which_chicker_command =
                TbotsProto_PowerPulseControl_ChickerControl_chip_pulse_width_tag;
            chicker.chicker_command.chip_pulse_width = chip_pulse_width;
            break;
        case ChickerCommandMode::KICK:
            control.chicker.which_chicker_command =
                TbotsProto_PowerPulseControl_ChickerControl_kick_pulse_width_tag;
            chicker.chicker_command.kick_pulse_width = kick_pulse_width;
            break;
        case ChickerCommandMode::AUTOCHIPORKICK:
            control.chicker.which_chicker_command =
                TbotsProto_PowerPulseControl_ChickerControl_auto_chip_or_kick_tag;
            switch (auto_chip_or_kick)
            {
                case AutoChipOrKickMode::AUTOCHIP:
                    chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick =
                        TbotsProto_PowerPulseControl_AutoChipOrKick_autochip_pulse_width_tag;
                    chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                        .autochip_pulse_width = autochip_pulse_width;
                    break;
                case AutoChipOrKickMode::AUTOKICK:
                    chicker.chicker_command.auto_chip_or_kick.which_auto_chip_or_kick =
                        TbotsProto_PowerPulseControl_AutoChipOrKick_autokick_pulse_width_tag;
                    chicker.chicker_command.auto_chip_or_kick.auto_chip_or_kick
                        .autokick_pulse_width = autokick_pulse_width;
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
std::unique_ptr<TbotsProto::PowerStatus> inline createTbotsPowerStatus(
    const TbotsProto_PowerStatus& status)
{
    auto buffer       = serializeToVector(status);
    auto proto_status = std::make_unique<TbotsProto::PowerStatus>();
    proto_status->ParseFromString(std::string(buffer.begin(), buffer.end()));
    return proto_status;
}
#endif
