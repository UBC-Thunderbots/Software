extern "C"
{
#include "move_primitive.h"
}
#include "firmware/app/primitives/test_util_world.h"

TEST_F(FirmwareTestUtilWorld, app_move_primitive_test_autochip)
{
    TbotsProto_Primitive primitive_msg;
    primitive_msg.which_primitive               = TbotsProto_Primitive_move_tag;
    TbotsProto_MovePrimitive move_primitive_msg = TbotsProto_MovePrimitive_init_zero;
    move_primitive_msg.dribbler_speed_rpm       = 1.0;
    move_primitive_msg.auto_chip_or_kick.which_auto_chip_or_kick =
        TbotsProto_AutoChipOrKick_autochip_distance_meters_tag;
    move_primitive_msg.auto_chip_or_kick.auto_chip_or_kick.autochip_distance_meters = 2.0;
    move_primitive_msg.max_speed_m_per_s                                            = 2.0;
    primitive_msg.primitive.move = move_primitive_msg;

    PrimitiveManager_t* manager = app_primitive_manager_create();

    app_primitive_manager_startNewPrimitive(manager, firmware_world, primitive_msg);
    // Checking `dribbler_speed_rpm` is passed into `app_dribbler_setSpeed` mock
    ASSERT_EQ(FirmwareTestUtil::set_requested_rpm_fake.arg0_val, 1.0);
    // Checking `chip_distance_meters` is passed into `app_chicker_enableAutokick` mock
    ASSERT_EQ(FirmwareTestUtil::enable_auto_chip_fake.arg0_val, 2.0);
    app_primitive_manager_destroy(manager);
}

TEST_F(FirmwareTestUtilWorld, app_move_primitive_test_autokick)
{
    TbotsProto_Primitive primitive_msg;
    primitive_msg.which_primitive               = TbotsProto_Primitive_move_tag;
    TbotsProto_MovePrimitive move_primitive_msg = TbotsProto_MovePrimitive_init_zero;
    move_primitive_msg.dribbler_speed_rpm       = 1.0;
    move_primitive_msg.auto_chip_or_kick.which_auto_chip_or_kick =
        TbotsProto_AutoChipOrKick_autokick_speed_m_per_s_tag;
    move_primitive_msg.auto_chip_or_kick.auto_chip_or_kick.autokick_speed_m_per_s = 3.0;
    move_primitive_msg.max_speed_m_per_s                                          = 4.0;
    primitive_msg.primitive.move = move_primitive_msg;

    PrimitiveManager_t* manager = app_primitive_manager_create();

    app_primitive_manager_startNewPrimitive(manager, firmware_world, primitive_msg);
    // Checking `dribbler_speed_rpm` is passed into `app_dribbler_setSpeed` mock
    ASSERT_EQ(FirmwareTestUtil::set_requested_rpm_fake.arg0_val, 1.0);
    // Checking `chip_distance_meters` is passed into `app_chicker_enableAutokick` mock
    ASSERT_EQ(FirmwareTestUtil::enable_auto_kick_fake.arg0_val, 3.0);
    app_primitive_manager_destroy(manager);
}
