#include "firmware/boards/robot_stm32h7/io/robot_status.h"

#include "firmware/boards/robot_stm32h7/io/proto_multicast.h"
#include "firmware/boards/robot_stm32h7/io/proto_multicast_communication_profile.h"
#include "shared/proto/robot_status_msg.nanopb.h"

static TbotsProto_RobotStatus g_robot_status_msg = TbotsProto_RobotStatus_init_zero;

void io_robot_status_task(void *argument)
{
    ProtoMulticastCommunicationProfile_t *comm_profile =
        (ProtoMulticastCommunicationProfile_t *)argument;

    for (;;)
    {
        // TODO (#1518) enable SNTP sys_now is currently only time since reset
        g_robot_status_msg.time_sent.epoch_timestamp_seconds = sys_now();

        // TODO (#1517) update robot id based on robot id selection
        g_robot_status_msg.robot_id                    = 0;
        g_robot_status_msg.firmware_status.fw_build_id = 0;

        g_robot_status_msg.break_beam_status = (TbotsProto_BreakBeamStatus){
            .ball_in_beam = false, .break_beam_reading = 0.0f};

        // TODO (#2080) update chipper and kicker fired values
        g_robot_status_msg.chipper_kicker_status = (TbotsProto_ChipperKickerStatus){
            .ms_since_chipper_fired = 0.0f, .ms_since_kicker_fired = 0.0f};

        // TODO (#2098) update motor rpms
        g_robot_status_msg.drive_units.front_left = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 4.0f, .wheel_position_rad = 0.0f};
        g_robot_status_msg.drive_units.back_left = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 4.0f, .wheel_position_rad = 0.0f};
        g_robot_status_msg.drive_units.front_right = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 4.0f, .wheel_position_rad = 0.0f};
        g_robot_status_msg.drive_units.back_right = (TbotsProto_DriveUnit){
            .wheel_ang_vel_rad_s = 4.0f, .wheel_position_rad = 0.0f};

        // TODO (#2081) update with dribbler temp
        g_robot_status_msg.temperature_status = (TbotsProto_TemperatureStatus){
            .dribbler_temperature = 100.0f, .board_temperature = 69.420f};

        // TODO (#2081) update with dribbler status
        g_robot_status_msg.dribbler_status =
            (TbotsProto_DribblerStatus){.dribbler_rpm = 1000.0f};

        // TODO (#1874) update with u-blox status
        g_robot_status_msg.network_status = (TbotsProto_NetworkStatus){
            .ms_since_last_vision_received = 0, .ms_since_last_primitive_received = 0};

        // TODO (#2097) update when power monitor has been fully integrated
        g_robot_status_msg.power_status = (TbotsProto_PowerStatus){
            .battery_voltage = 200.0f, .capacitor_voltage = 0.0f};

        io_proto_multicast_communication_profile_acquireLock(comm_profile);

        memcpy(io_proto_multicast_communication_profile_getProtoStruct(comm_profile),
               &g_robot_status_msg, sizeof(TbotsProto_RobotStatus));

        io_proto_multicast_communication_profile_releaseLock(comm_profile);
        io_proto_multicast_communication_profile_notifyEvents(comm_profile,
                                                              PROTO_UPDATED);
        osDelay(100);
    }
}
