package frc.robot.commands.shooter;

import static frc.robot.constants.GlobalConstants.*;
import static frc.robot.constants.ShooterConstants.*;
import static frc.robot.constants.VisionConstants.*;
import static frc.robot.utilities.CustomUnits.RotationsPerMinute;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Swerve swerveDrive;
    private final Vision vision;
    private final Hopper hopper;

    private double lastDistanceToHub = -1;
    private double lastShooterMapRPM = -1;

    public ShootCommand(Shooter shooter, Swerve swerveDrive, Vision vision, Hopper hopper) {
        this.shooter = shooter;
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.hopper = hopper;

        addRequirements(shooter, hopper);
    }

    @Override
    public void initialize() {
        hopper.runHopper();
        shooter.stopFeeder();
    }

    @Override
    public void execute() {
        if (shooter.shooterIsUpToSpeed()) {
            shooter.runFeeder();
        }

        int hubAprilTagID = isAllianceBlue() ? 25 : 10;

        var targetHubYaw = vision.getTX(hubAprilTagID);

        if (targetHubYaw.isEmpty()) {
            swerveDrive.setAlignStatus(false, 0);
            SmartDashboard.putNumber("Yaw from target", -1);

            if (lastShooterMapRPM > 0 && lastDistanceToHub >= 1.5) {
                // If our last distance detected was within the range of DISTANCE_VS_RPM_MAP
                // and we have detected the tag at least once
                // we keep shooting at that speed so the shooter doesn't stutter
                // (i.e. if we lose detection for a second or two)
                shooter.setFlywheelSpeed(RotationsPerMinute.of(lastShooterMapRPM));
            } else if (vision.getBestTarget() != null
                    && NEUTRAL_ZONE_APRILTAG_IDS.contains(vision.getBestTarget().getFiducialId())) {
                // If robot is in the neutral zone / sees a neutral zone Apriltag
                // we shoot faster to be able to pass to our side
                shooter.setFlywheelSpeed(RotationsPerMinute.of(PASSING_SHOOTER_RPM));
            } else {
                // Otherwise (if we're right up against the hub)
                // shoot at minimum RPM
                shooter.setFlywheelSpeed(RotationsPerMinute.of(MINIMUM_SHOOTER_RPM));
            }
        } else {
            swerveDrive.setAlignStatus(true, targetHubYaw.get());
            if (vision.getDistance(hubAprilTagID).isPresent()) {
                lastDistanceToHub = vision.getDistance(hubAprilTagID).get();
            }

            SmartDashboard.putNumber("Yaw from target", targetHubYaw.get());
            SmartDashboard.putNumber("Distance from hub", lastDistanceToHub);

            if (Math.abs(targetHubYaw.get()) <= YAW_ACCEPTABLE_ERROR) {
                if (lastDistanceToHub == -1) {
                    shooter.setFlywheelSpeed(RotationsPerMinute.of(MINIMUM_SHOOTER_RPM));
                }
                lastShooterMapRPM = DISTANCE_VS_RPM_MAP.get(lastDistanceToHub);
                shooter.setFlywheelSpeed(RotationsPerMinute.of(lastShooterMapRPM));
            } else {
                shooter.stopShooter();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.setAlignStatus(false, 0);
        swerveDrive.drive(new ChassisSpeeds(), true);
        shooter.stopShooter();
        hopper.stopHopper();
    }
}
