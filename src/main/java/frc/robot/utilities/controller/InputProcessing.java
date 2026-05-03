package frc.robot.utilities.controller;

import static frc.robot.constants.GlobalConstants.OperatorConstants.DEADBAND_THRESHOLD;

public class InputProcessing {
    /**
     * @param rawInput The input that the deadband should be applied to
     * @return The given input where if the magnitude is below the deadband threshold it is equal to
     * zero
     */
    public static double applyDeadband(double rawInput) {
        return Math.abs(rawInput) < DEADBAND_THRESHOLD ? 0 : rawInput;
    }

    /**
     * Applies a squared curve to the input, retaining the original sign.
     * <p>
     * This creates a quadratic response curve,
     * providing finer control, while still
     * allowing for maximum output.
     * </p>
     *
     * @param input The raw input value from the controller.
     * @return The scaled input value, retaining the original sign.
     */
    public static double curve(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), 2);
    }
}
