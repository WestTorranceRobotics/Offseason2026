package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;

        public Rotation2d rotation2D = Rotation2d.kZero;
        public AngularVelocity angularVelocity = RadiansPerSecond.of(0);
    }

    public default void updateInputs(GyroIOInputs inputd) {}
}
