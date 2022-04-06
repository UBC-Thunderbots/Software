#pragma once

/**
 * This class represents the input from a generic handheld game controller
 * (like an Xbox of Playstation controller) used to control a robot
 */
class ControllerInput
{
   public:
    explicit ControllerInput();

    /**
     * Returns a value in the range [-1.0, 1.0] representing the desired amount of
     * motion along the x-axis is robot local coordinates.
     *
     * @return a value in the range [-1.0, 1.0] representing the desired amount of
     * motion along the x-axis is robot local coordinates.
     */
    double getLinearMotionX() const;

    /**
     * Returns a value in the range [-1.0, 1.0] representing the desired amount of
     * motion along the y-axis is robot local coordinates.
     *
     * @return a value in the range [-1.0, 1.0] representing the desired amount of
     * motion along the y-axis is robot local coordinates.
     */
    double getLinearMotionY() const;

    /**
     * Returns a value in the range [-1.0, 1.0] representing the desired amount of
     * angular motion in robot local coordinates.
     *
     * @return a value in the range [-1.0, 1.0] representing the desired amount of
     * angular motion in robot local coordinates.
     */
    double getAngularMotion() const;

    /**
     * Returns whether or not the button mapped to kicking is pressed
     *
     * @return true if the button mapped to kicking is pressed, and false otherwise
     */
    bool isKickButtonPressed() const;

    /**
     * Returns whether or not the button mapped to chipping is pressed
     *
     * @return true if the button mapped to chipping is pressed, and false otherwise
     */
    bool isChipButtonPressed() const;

    /**
     * Returns whether or not the button mapped to dribbling is pressed
     *
     * @return true if the button mapped to dribbling is pressed, and false otherwise
     */
    bool isDribblerButtonPressed() const;

    /**
     * Sets the value of the linear motion along the x-axis in robot local coordinates
     *
     * @pre The value must be in the range [0, 1]
     * @param value The new value
     */
    void setLinearMotionX(double value);

    /**
     * Sets the value of the linear motion along the y-axis in robot local coordinates
     *
     * @pre The value must be in the range [0, 1]
     * @param value The new value
     */
    void setLinearMotionY(double value);

    /**
     * Sets the value of the angular motion along in robot local coordinates
     *
     * @pre The value must be in the range [0, 1]
     * @param value The new value
     */
    void setAngularMotion(double value);

    /**
     * Sets whether or not the kicking button is pressed
     *
     * @param pressed The new value
     */
    void setKickButtonPressed(bool pressed);

    /**
     * Sets whether or not the chipping button is pressed
     *
     * @param pressed The new value
     */
    void setChipButtonPressed(bool pressed);

    /**
     * Sets whether or not the dribbling button is pressed
     *
     * @param pressed The new value
     */
    void setDribblerButtonPressed(bool pressed);

   private:
    double linear_motion_x;
    double linear_motion_y;
    double angular_motion;
    bool kick_button_pressed;
    bool chip_button_pressed;
    bool dribbler_button_pressed;
};
