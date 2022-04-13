#include <pb_encode.h>
#include <proto/primitive.nanopb.h>

#ifdef PLATFORMIO_BUILD
#include <power_frame_msg_platformio.h>
#else
#include "pb_decode.h"
#include "pb_encode.h"
#include "proto/message_translation/power_frame_msg.h"

#endif

std::vector<uint8_t> serializeToVector(const TbotsProto_PowerFrame& data)
{
    std::vector<uint8_t> buffer(TbotsProto_PowerFrame_size);
    pb_ostream_t stream =
        pb_ostream_from_buffer(static_cast<uint8_t*>(buffer.data()), buffer.size());
    if (!pb_encode(&stream, TbotsProto_PowerFrame_fields, &data))
    {
        throw std::runtime_error(
            "Failed to encode PowerFrame msg to vector when converting nanopb to vector");
    }
    return buffer;
}

std::vector<uint8_t> serializeToVector(const TbotsProto_PowerControl& data)
{
    std::vector<uint8_t> buffer(TbotsProto_PowerControl_size);
    pb_ostream_t stream =
        pb_ostream_from_buffer(static_cast<uint8_t*>(buffer.data()), buffer.size());
    if (!pb_encode(&stream, TbotsProto_PowerControl_fields, &data))
    {
        throw std::runtime_error(
            "Failed to encode PowerControl msg to vector when converting nanopb to vector");
    }
    return buffer;
}

std::vector<uint8_t> serializeToVector(const TbotsProto_PowerStatus& data)
{
    std::vector<uint8_t> buffer(TbotsProto_PowerStatus_size);
    pb_ostream_t stream =
        pb_ostream_from_buffer(static_cast<uint8_t*>(buffer.data()), buffer.size());
    if (!pb_encode(&stream, TbotsProto_PowerStatus_fields, &data))
    {
        throw std::runtime_error(
            "Failed to encode PowerStatus msg to vector when converting nanopb to vector");
    }
    return buffer;
}

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

TbotsProto_PowerStatus createNanoPbPowerStatus(float battery_voltage,
                                               float capacitor_voltage,
                                               float current_draw,
                                               float high_voltage_measurement_volts,
                                               float geneva_angle_deg,
                                               bool breakbeam_tripped, bool flyback_fault)
{
    TbotsProto_PowerStatus status         = TbotsProto_PowerStatus_init_default;
    status.battery_voltage                = battery_voltage;
    status.capacitor_voltage              = capacitor_voltage;
    status.current_draw                   = current_draw;
    status.high_voltage_measurement_volts = high_voltage_measurement_volts;
    status.geneva_angle_deg               = geneva_angle_deg;
    status.breakbream_tripped             = breakbeam_tripped;
    status.flyback_fault                  = flyback_fault;
    return status;
}

#ifndef PLATFORMIO_BUILD
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
TbotsProto_PowerControl createNanoPbPowerControl(
    ChickerCommandMode chicker_command, float kick_speed_m_per_s,
    float chip_distance_meters, AutoChipOrKickMode auto_chip_or_kick,
    float autochip_distance_meters, float autokick_speed_m_per_s, float angle_deg,
    float rotation_speed_rpm, TbotsProto_PowerControl_ChargeMode charge_mode)
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
        case ChickerCommandMode::KICK:
            control.chicker.which_chicker_command =
                TbotsProto_PowerControl_ChickerControl_kick_speed_m_per_s_tag;
            chicker.chicker_command.kick_speed_m_per_s = kick_speed_m_per_s;
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
    }

    TbotsProto_PowerControl_GenevaControl geneva =
        TbotsProto_PowerControl_GenevaControl_init_default;
    geneva.angle_deg          = angle_deg;
    geneva.rotation_speed_rpm = rotation_speed_rpm;

    control.chicker     = chicker;
    control.geneva      = geneva;
    control.charge_mode = charge_mode;

    return control;
}
std::unique_ptr<TbotsProto::PowerStatus> createTbotsPowerStatus(
    const TbotsProto_PowerStatus& status)
{
    auto buffer       = serializeToVector(status);
    auto proto_status = std::make_unique<TbotsProto::PowerStatus>();
    proto_status->ParseFromString(std::string(buffer.begin(), buffer.end()));
    return proto_status;
}
#endif
