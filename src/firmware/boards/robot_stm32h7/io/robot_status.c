#include "firmware/boards/robot_stm32h7/io/robot_status.h"

#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "shared/proto/robot_status_msg.nanopb.h"

static TbotsProto_RobotStatus g_robot_status_msg;

void io_robot_status_task(void *argument)
{
    ProtoMulticastCommunicationProfile_t *comm_profile =
        (ProtoMulticastCommunicationProfile_t *)argument;

    for (;;)
    {
        // TODO enable SNTP sys_now is currently only time since reset
        // https://github.com/UBC-Thunderbots/Software/issues/1518
        g_robot_status_msg.time_sent.epoch_timestamp_seconds = sys_now();
        g_robot_status_msg.robot_id                          = 0;
        g_robot_status_msg.firmware_status.fw_build_id       = 0;

        g_robot_status_msg.break_beam_status = (TbotsProto_BreakBeamStatus){
            .ball_in_beam = false, .break_beam_reading = 0.0f};

        g_robot_status_msg.chipper_kicker_status = (TbotsProto_ChipperKickerStatus){
            .ms_since_chipper_fired = 0.0f, .ms_since_kicker_fired = 0.0f};

        g_robot_status_msg.drive_units.front_left = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 0.0f, .wheel_position_rad = 0.0f};
        g_robot_status_msg.drive_units.back_left = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 0.0f, .wheel_position_rad = 0.0f};
        g_robot_status_msg.drive_units.front_right = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 0.0f, .wheel_position_rad = 0.0f};
        g_robot_status_msg.drive_units.back_right = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 0.0f, .wheel_position_rad = 0.0f};

        g_robot_status_msg.temperature_status = (TbotsProto_TemperatureStatus){
            .dribbler_temperature = 0.0f, .board_temperature = 0.0f};

        g_robot_status_msg.dribbler_status =
            (TbotsProto_DribblerStatus){.dribbler_rpm = 0.0f};

        g_robot_status_msg.network_status = (TbotsProto_NetworkStatus){
            .ms_since_last_vision_received = 0, .ms_since_last_primitive_received = 0};

        g_robot_status_msg.power_status =
            (TbotsProto_PowerStatus){.battery_voltage = 0.0f, .capacitor_voltage = 0.0f};

        // We change the power status values randomly so that robot diagnostics
        // can "see" this robot on the network. This is a stopgap until we have
        // actual values for RobotStatus
        io_proto_multicast_communication_profile_acquireLock(comm_profile);

        memcpy(io_proto_multicast_communication_profile_getProtoStruct(comm_profile),
               &g_robot_status_msg, sizeof(TbotsProto_RobotStatus));

        io_proto_multicast_communication_profile_releaseLock(comm_profile);
        io_proto_multicast_communication_profile_notifyEvents(comm_profile,
                                                              PROTO_UPDATED);
        // run loop at 10hz
        osDelay(100);
    }
}
