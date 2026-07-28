package frc.robot.utilities.controller;

import org.wpilib.command2.button.Trigger;

public interface Controller {
    /**
     * @return The amount of x on the left joystick.
     */
    double getLeftX();

    /**
     * @return The amount of y on the left joystick.
     */
    double getLeftY();

    /**
     * @return The amount of x on the right joystick.
     */
    double getRightX();

    /**
     * @return The amount of y on the right joystick.
     */
    double getRightY();

    /**
     * @Trigger The amount of hold on the right analog trigger.
     */
    double getRightAnalogTrigger();

    /**
     * @Trigger Triggers when the bottom button is pressed.
     */
    Trigger aOrCross();

    /**
     * @Trigger Triggers when the right button is pressed.
     */
    Trigger bOrCircle();

    /**
     * @Trigger Triggers when the up button is pressed.
     */
    Trigger yOrTriangle();

    /**
     * @Trigger Triggers when the left button is pressed.
     */
    Trigger xOrSquare();

    /**
     * @Trigger Triggers when the left d-pad button is pressed.
     */
    Trigger dPadLeft();

    /**
     * @Trigger Triggers when the up d-pad button is pressed.
     */
    Trigger dPadUp();

    /**
     * @Trigger Triggers when the right d-pad button is pressed.
     */
    Trigger dPadRight();

    /**
     * @Trigger Triggers when the down d-pad button is pressed.
     */
    Trigger dPadDown();

    /**
     * @Trigger Triggers a zero of the swerve on the right analog trigger.
     */
    Trigger zero();
}
