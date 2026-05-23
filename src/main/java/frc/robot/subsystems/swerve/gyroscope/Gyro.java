package frc.robot.subsystems.swerve.gyroscope;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class Gyro {
    private final GyroIO gyroIO;

    public Gyro(GyroIO gyroIO) {
        this.gyroIO = gyroIO;
    }

    public Rotation2d getRotation() {
        return gyroIO.getRotation();
    }

    public AngularVelocity getAngularVelocity() {
        return gyroIO.getAngularVelocity();
    }

    public void updateInputs() {
        gyroIO.updateInputs();
    }
}
