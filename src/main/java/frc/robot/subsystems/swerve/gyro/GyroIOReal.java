package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;

public class GyroIOReal implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroIOReal() {
        this.pigeon = new Pigeon2(RealRobotConstants.PIGEON2_ID);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = pigeon.isConnected();
        inputs.rotation2D = pigeon.getRotation2d();
        inputs.angularVelocity = pigeon.getAngularVelocityZWorld().getValue();
    }
}
