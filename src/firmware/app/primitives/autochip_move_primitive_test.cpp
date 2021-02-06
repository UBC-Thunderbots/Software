extern "C"
{
#include "autochip_move_primitive.h"
}
#include "firmware/app/primitives/test_util_world.h"

TEST_F(FirmwareTestUtilWorld, app_autochip_move_primitive_test)
{
    TbotsProto_Primitive primitive_msg;
    primitive_msg.which_primitive = TbotsProto_Primitive_autochip_move_tag;
    TbotsProto_AutochipMovePrimitive autochip_primitive_msg;
    autochip_primitive_msg.dribbler_speed_rpm   = 1.0;
    autochip_primitive_msg.chip_distance_meters = 2.0;
    primitive_msg.primitive.autochip_move       = autochip_primitive_msg;

    PrimitiveManager_t* manager = app_primitive_manager_create();

    app_primitive_manager_startNewPrimitive(manager, firmware_world, primitive_msg);
    // Checking `dribbler_speed_rpm` is passed into `app_dribbler_setSpeed` mock
    ASSERT_EQ(FirmwareTestUtil::set_requested_rpm_fake.arg0_val, 1.0);
    // Checking `chip_distance_meters` is passed into `app_chicker_enableAutokick` mock
    ASSERT_EQ(FirmwareTestUtil::enable_auto_chip_fake.arg0_val, 2.0);
    app_primitive_manager_destroy(manager);
}
