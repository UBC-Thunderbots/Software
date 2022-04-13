#pragma once

#include <cstdint>
#include <vector>
#include <memory>

#ifdef PLATFORMIO_BUILD
#include <proto/power_frame_msg.nanopb.h>
#else
#include "proto/power_frame_msg.pb.h"
#include "proto/primitive/primitive_types.h"

extern "C"
{
#include "proto/power_frame_msg.nanopb.h"
}
#endif

// The nanopb generated size isn't c++ compatible so we redefine it here
#undef TbotsProto_PowerFrame_size
#define TbotsProto_PowerFrame_size std::max(TbotsProto_PowerControl_size, TbotsProto_PowerStatus_size) \
        + 2 * sizeof(uint32_t) + sizeof(uint16_t)

/**
 * Serialize nanopb into its byte representation
 *
 * @param data nanopb msg to be serialized
 * @return byte representation of provided nanopb
 */
std::vector<uint8_t> serializeToVector(const TbotsProto_PowerFrame& data);
std::vector<uint8_t> serializeToVector(const TbotsProto_PowerControl& data);
std::vector<uint8_t> serializeToVector(const TbotsProto_PowerStatus& data);

/**
 * Helper function to set the type and value of a power frame's power_msg
 *
 * @param frame frame to set the power_msg of
 * @param control/status power_msg to set
 */
void setPowerMsg(TbotsProto_PowerFrame& frame, const TbotsProto_PowerControl& control);
void setPowerMsg(TbotsProto_PowerFrame& frame, const TbotsProto_PowerStatus& status);
/**
 * Creates a nanopb power status msg with provided fields
 *
 * @return a nanopb power status msg with provided fields
 */
TbotsProto_PowerStatus createNanoPbPowerStatus(float battery_voltage, float capacitor_voltage,
                                               float current_draw, float high_voltage_measurement_volts, float geneva_angle_deg,
                                               bool breakbeam_tripped, bool flyback_fault);
// These functions are only used in power service
// Protobuf needs additional support to run on an esp32
#ifndef PLATFORMIO_BUILD
/**
 * Converts a google protobuf power control msg to its nanopb representation
 *
 * @param google_control protobuf message to convert
 * @return a nanopb power control msg matching provided protobuf
 */
TbotsProto_PowerControl createNanoPbPowerControl(const TbotsProto::PowerControl& google_control);
/**
 * Creates a nanopb power control msg with provided fields
 * Note: Currently only used for testing
 *
 * @return a nanobp power control msg with provided fields
 */
TbotsProto_PowerControl createNanoPbPowerControl(ChickerCommandMode chicker_command, float kick_speed_m_per_s, float chip_distance_meters,
                                                 AutoChipOrKickMode auto_chip_or_kick, float autochip_distance_meters,
                                                 float autokick_speed_m_per_s, float angle_deg, float rotation_speed_rpm,
                                                 TbotsProto_PowerControl_ChargeMode charge_mode);
/**
 * Converts a nanopb power status msg to its google protobuf representation
 *
 * @param status nanopb message to convert
 * @return a google protobuf power status matching provided nanopb
 */
std::unique_ptr<TbotsProto::PowerStatus> createTbotsPowerStatus(const TbotsProto_PowerStatus& status);
#endif