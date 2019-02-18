cmake_minimum_required(VERSION 3.8.2)
project(thunderbots_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
    FILES
    Ball.msg
    Circle2D.msg
    DrawLayer.msg
    DrawShape.msg
    Field.msg
    Pass.msg
    Point2D.msg
    Primitive.msg
    PrimitiveArray.msg
    RefboxCommand.msg
    RefboxData.msg
    RefboxTeamInfo.msg
    Robot.msg
    Team.msg
    World.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS message_runtime std_msgs
)

