package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
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

    /**
     * @return Voltage that is applied to the drive motor
     */
    public Voltage getDriveVoltage();

    /**
     * @return Voltage that is applied to the steer motor
     */
    public Voltage getSteerVoltage();

    public AngularVelocity getSteerVelocity();

    /**
     * @return The direction the wheel is facing
     */
    public Rotation2d getSteerAngle();

    /**
     * @return The rotation of the wheel (this can be greater than one rotation)
     */
    public Angle getDriveWheelPosition();

    public AngularVelocity getDriveWheelVelocity();

    public default void updateInputs(ModuleIOInputs inputs) {}
    ;
}
