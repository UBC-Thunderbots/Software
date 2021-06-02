#pragma once

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
