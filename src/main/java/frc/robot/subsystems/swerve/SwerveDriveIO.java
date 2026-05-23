package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;

public interface SwerveDriveIO {
    public default void updateInputs() {}
    ;

    public default void setSimulationWorldPose(Pose2d pose) {}
    ;
}
