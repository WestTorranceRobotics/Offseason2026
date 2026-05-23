package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;

public interface SwerveDriveIO {
    default void updateInputs() {}

    default void setSimulationWorldPose(Pose2d pose) {}
}
