package frc.robot.subsystems.swerve.module;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public double driveVoltage = 0;
        public double steerVoltage = 0;

        public double driveWheelPositionRotations = 0;
        public double driveWheelVelocityRPS = 0;

        public double steerAngleRad = 0;
        public double steerVelocityRadPerSec = 0;
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

    public default void updateInputs(ModuleIOInputs inputs) {}
    ;
}
