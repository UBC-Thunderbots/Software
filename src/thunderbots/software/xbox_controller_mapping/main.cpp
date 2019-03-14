#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <thunderbots_msgs/Primitive.h>
#include <thunderbots_msgs/PrimitiveArray.h>
#include <thunderbots_msgs/Robot.h>
#include <thunderbots_msgs/Team.h>
#include <thunderbots_msgs/World.h>

#include "ai/primitive/chip_primitive.h"
#include "ai/primitive/direct_velocity_primitive.h"
#include "ai/primitive/kick_primitive.h"
#include "shared/constants.h"
#include "util/constants.h"
#include "util/parameter/dynamic_parameter_utils.h"
#include "util/parameter/dynamic_parameters.h"
#include "util/ros_messages.h"

// Member variables we need to maintain state
// They are kept in an anonymous namespace so they are not accessible outside this
// file and are not created as global static variables.
namespace
{
    // The publisher to send our custom Primitive commands based on the controller
    // information
    ros::Publisher primitive_publisher;
    // Publishers to mock network_input data
    ros::Publisher world_publisher;

    // Button mappings for a Microsoft Xbox 360 Wired Controller for Linux
    // http://wiki.ros.org/joy section 5.3
    enum ControllerButtons_t
    {
        A = 0,
        B,
        X,
        Y,
        LEFT_BUMPER,
        RIGHT_BUMPER,
        BACK,
        START,
        POWER,
        BUTTON_STICK_LEFT,
        BUTTON_STICK_RIGHT
    } ControllerButtons;

    // Axis mappings for a Microsoft Xbox 360 Wired Controller for Linux
    // http://wiki.ros.org/joy section 5.3
    enum ControllerAxes_t
    {
        LEFTRIGHT_AXIS_STICK_LEFT = 0,
        UPDOWN_AXIS_STICK_LEFT,
        LEFT_TRIGGER,
        LEFTRIGHT_AXIS_STICK_RIGHT,
        UPDOWN_AXIS_STICK_RIGHT,
        RIGHT_TRIGGER,
        CROSS_PAD_LEFT_RIGHT,
        CROSS_PAD_UP_DOWN
    } ControllerAxes;
}  // namespace

/**
 * Toggles whether or not the dribbler is running every time the given button is pressed
 * (value of 1 for pressed, 0 for not pressed). Acts like a D Flip-Flop.
 *
 * @param button_pressed The value indicating if the button is pressed. 1 indicates the
 * button is pressed, 0 indicates the button is not pressed
 *
 * @return 1 if the dribbler should be enabled, and 0 if the dribbler should not be
 * enabled
 */
unsigned int toggleDribble(unsigned int button_pressed)
{
    static unsigned int dribble_enabled  = 0;
    static bool button_has_been_released = true;

    if (button_pressed == 1 && button_has_been_released)
    {
        dribble_enabled          = dribble_enabled == 1 ? 0 : 1;
        button_has_been_released = false;
    }
    else if (button_pressed == 0 && !button_has_been_released)
    {
        button_has_been_released = true;
    }

    return dribble_enabled;
}

void joystickUpdateCallback(const sensor_msgs::Joy::ConstPtr &msg)
{
    /************************************************************************************
     *        Read XBox Controller Information and store relevant variables             *
     ************************************************************************************/
    sensor_msgs::Joy::_axes_type axes       = msg->axes;
    sensor_msgs::Joy::_buttons_type buttons = msg->buttons;

    // Axes values report values in the range [-1.0, 1.0]. Values are 1.0 when the
    // joystick is pushed all the way up, -1.0 when the joystick is all the way down,
    // 1.0 when the joystick is pushed all the way left, and -1.0 when the joystick is
    // pushed all the way right
    double robot_x_velocity =
        axes[UPDOWN_AXIS_STICK_LEFT] * ROBOT_MAX_SPEED_METERS_PER_SECOND *
        Util::DynamicParameters::XBoxControllerDemo::linear_sensitivity.value();
    double robot_y_velocity =
        axes[LEFTRIGHT_AXIS_STICK_LEFT] * ROBOT_MAX_SPEED_METERS_PER_SECOND *
        Util::DynamicParameters::XBoxControllerDemo::linear_sensitivity.value();
    double robot_angular_velocity =
        axes[LEFTRIGHT_AXIS_STICK_RIGHT] * ROBOT_MAX_ANG_SPEED_RAD_PER_SECOND *
        Util::DynamicParameters::XBoxControllerDemo::angular_sensitivity.value();

    // The Y button reports an integer of 1 when pressed, and 0 when not pressed
    double robot_dribbler_rpm =
        toggleDribble(static_cast<unsigned int>(buttons[Y])) *
        Util::DynamicParameters::XBoxControllerDemo::dribbler_rpm.value();

    // The triggers report values in the range [-1.0, 1.0]. Values are < 0 when the
    // trigger is pressed/held down more than halfway, and > 0 otherwise
    double robot_kick_speed_meters_per_second =
        Util::DynamicParameters::XBoxControllerDemo::kick_speed_meters_per_second.value();
    if (axes[RIGHT_TRIGGER] > 0.0)
    {
        robot_kick_speed_meters_per_second = 0.0;
    }

    double robot_chip_distance_meters =
        Util::DynamicParameters::XBoxControllerDemo::chip_distance_meters.value();
    if (axes[LEFT_TRIGGER] > 0.0)
    {
        robot_chip_distance_meters = 0.0;
    }

    /************************************************************************************
     *               Create and publish mock data for the World state                   *
     ************************************************************************************/
    // We spoof the information about the state of the robot because it is unnecessary for
    // the XBox controller demos. These demos typically run where we have no vision, so we
    // just simply report the robot is always at Point(0, 0), and just set the velocities
    // directly. The network_input node can't be used since there is no vision data
    Robot robot =
        Robot(static_cast<unsigned int>(
                  Util::DynamicParameters::XBoxControllerDemo::robot_id.value()),
              Point(), Vector(), Angle::zero(), AngularVelocity::zero(),
              Timestamp::fromSeconds(0));
    Team team = Team(Duration::fromMilliseconds(
        Util::Constants::ROBOT_DEBOUNCE_DURATION_MILLISECONDS));
    team.updateRobots({robot});
    thunderbots_msgs::Team friendly_team_msg =
        Util::ROSMessages::convertTeamToROSMessage(team);

    // We spoof the information about the state of the ball for similar reasons to the
    // robot above. Since the AI is not running (a human is controlling the robot) we
    // don't need to report the real ball information.
    Ball ball                       = Ball(Point(), Vector(), Timestamp::fromSeconds(0));
    thunderbots_msgs::Ball ball_msg = Util::ROSMessages::convertBallToROSMessage(ball);

    // Publish our spoofed World
    thunderbots_msgs::World world_msg;
    world_msg.friendly_team = friendly_team_msg;
    world_msg.ball          = ball_msg;
    world_publisher.publish(world_msg);

    /************************************************************************************
     *       Create and publish Primitives using the Controller information             *
     ************************************************************************************/
    // Create and send the DirectVelocityPrimitive constructed from the commands received
    // from the XBox Controller
    std::unique_ptr<Primitive> primitive = std::make_unique<DirectVelocityPrimitive>(
        static_cast<unsigned int>(
            Util::DynamicParameters::XBoxControllerDemo::robot_id.value()),
        robot_x_velocity, robot_y_velocity, robot_angular_velocity, robot_dribbler_rpm);

    // Send a KickPrimitive if we want to kick
    if (robot_kick_speed_meters_per_second != 0.0)
    {
        primitive = std::make_unique<KickPrimitive>(
            Util::DynamicParameters::XBoxControllerDemo::robot_id.value(),
            robot.position(), robot.orientation(), robot_kick_speed_meters_per_second);
    }

    // Send a ChipPrimitive if we want to chip
    if (robot_chip_distance_meters != 0.0)
    {
        primitive = std::make_unique<ChipPrimitive>(
            Util::DynamicParameters::XBoxControllerDemo::robot_id.value(),
            robot.position(), robot.orientation(), robot_chip_distance_meters);
    }

    // Publish the primitive
    thunderbots_msgs::Primitive primitive_msg = primitive->createMsg();
    thunderbots_msgs::PrimitiveArray primitive_array;
    primitive_array.primitives.emplace_back(primitive_msg);
    primitive_publisher.publish(primitive_array);
}

int main(int argc, char **argv)
{
    // Init ROS node
    ros::init(argc, argv, "xbox_controller_mapping");
    ros::NodeHandle node_handle;

    // Create publishers
    primitive_publisher = node_handle.advertise<thunderbots_msgs::PrimitiveArray>(
        Util::Constants::AI_PRIMITIVES_TOPIC, 1);
    world_publisher = node_handle.advertise<thunderbots_msgs::World>(
        Util::Constants::NETWORK_INPUT_WORLD_TOPIC, 1);

    // Create subscribers
    ros::Subscriber joy_node_subscriber =
        node_handle.subscribe(Util::Constants::JOY_NODE_TOPIC, 1, joystickUpdateCallback);

    // Initialize Dynamic Parameters
    auto update_subscribers =
        Util::DynamicParameters::initUpdateSubscriptions(node_handle);

    // Services any ROS calls in a separate thread "behind the scenes". Does not return
    // until the node is shutdown
    // http://wiki.ros.org/roscpp/Overview/Callbacks%20and%20Spinning
    ros::spin();

    return 0;
}
