#pragma once
#include "firmware/boards/robot_stm32h7/adc.h"
#include "firmware/boards/robot_stm32h7/tim.h"

typedef struct
{
    /**
     * \brief The X component of the robot’s accumulated motion.
     */
    float x;

    /**
     * \brief The Y component of the robot’s accumulated motion.
     */
    float y;

    /**
     * \brief The angular component of the robot’s accumulated motion.
     */
    float angle;

    /**
     * \brief The X component of the robot’s velocity.
     */
    float vx;

    /**
     * \brief The Y component of the robot’s velocity.
     */
    float vy;

    /**
     * \brief The robot’s angular velocity.
     */
    float avel;

} dr_data_t;

typedef struct
{
    /**
     * \brief The X component of the ball’s accumulated motion.
     */
    float x;

    /**
     * \brief The Y component of the ball’s accumulated motion.
     */
    float y;

    /**
     * \brief The X component of the ball’s velocity.
     */
    float vx;

    /**
     * \brief The Y component of the ball’s velocity.
     */
    float vy;

} dr_ball_data_t;


typedef struct
{
    /**
     * \brief The x component of the robot's global position in metres.
     */
    float x;

    /**
     * \brief The y component of the robot's global position in metres.
     */
    float y;

    /**
     * \brief The theta component of the robot's position in the global frame in rad.
     */
    float angle;

    /**
     * \brief The timestamp associated with this camera frame.
     */
    uint64_t timestamp;


} robot_camera_data_t;


typedef struct
{
    /**
     * \brief The x component of the ball's global position in metres.
     */
    float x;

    /**
     * \brief The y component of the ball's global position in metres.
     */
    float y;

    /**
     * \brief The timestamp associated with this camera frame.
     */
    uint64_t timestamp;

} ball_camera_data_t;

/*
 * @returns the x,y positions of the ball
 */
float io_vision_getBallPositionX(void);
float io_vision_getBallPositionY(void);

/*
 * @returns the x,y components of the ball's velocity
 */
float io_vision_getBallVelocityX(void);
float io_vision_getBallVelocityY(void);

/*
 * @returns the x,y positions of this robot
 */
float io_vision_getRobotPositionX(void);
float io_vision_getRobotPositionY(void);

/*
 * @returns the x,y components of this robots velocity
 */
float io_vision_getRobotVelocityX(void);
float io_vision_getRobotVelocityY(void);

/*
 * @returns the orientation of this robot
 */
float io_vision_getRobotOrientation(void);

/*
 * @returns the angular velocity of this robot
 */
float io_vision_getRobotAngularVelocity(void);

// TODO DOCS DR
void io_vision_init(ADC_HandleTypeDef* adc);
void io_vision_task(void* arg);
void io_lock_vision();
void io_unlock_vision();
void io_vision_stepDeadReckoning();
void io_vision_applyVisionFrameToDeadReckoning(uint32_t robot_id);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ErrorCallback(ADC_HandleTypeDef* hadc);
