package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVoltage = 0;
        public double steerVoltage = 0;

        public double driveWheelPositionRotations = 0;
        public double driveWheelVelocityRPS = 0;

        public double steerAngleRad = 0;
        public double steerVelocityRadPerSec = 0;

        public double driveCurrent = 0;
    }

    /**
     * Set voltage supplied to drive motor
     *
     * @param voltage Voltage to be supplied to drive motor
     */
    public void setDriveVoltage(Voltage voltage);

    /**
     * Set voltage supplied to steer motor
     *
     * @param voltage Voltage to be supplied to steer motor
     */
    public void setSteerVoltage(Voltage voltage);

    /**
     * Set closed-loop velocity setpoint for the drive wheel.
     *
     * <p>Each implementation handles actuation its own
     * way: the real IO sends a TalonFX velocity control request (converting wheel velocity to
     * rotor velocity with the gear ratio), while the sim IO runs a software PID + feedforward
     * loop that outputs a voltage.
     *
     * @param wheelVelocity Desired velocity of the drive wheel
     */
    public void setDriveVelocity(AngularVelocity wheelVelocity);

    public default void updateInputs(ModuleIOInputs inputs) {}
    ;
}
