package frc.robot.subsystems.swerve.gyroscope;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;

public class GyroIOReal implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOReal() {
        this(RealRobotConstants.PIGEON2_ID);
    }

    public GyroIOReal(int id) {
        this.pigeon = new Pigeon2(id);
    }

    @Override
    public Rotation2d getRotation() {
        return pigeon.getRotation2d();
    }

    @Override
    public AngularVelocity getAngularVelocity() {
        // TODO this value must be checked to be correct
        return pigeon.getAngularVelocityZWorld().getValue();
    }
}
