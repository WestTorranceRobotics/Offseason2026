package frc.robot.subsystems.swerve.gyro;

import static org.wpilib.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.units.measure.AngularVelocity;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        public Rotation2d rotation2D = Rotation2d.kZero;
        public AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}
