package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.*;
import static frc.robot.utilities.CustomUnits.RotationsPerMinute;
import static org.ironmaple.utils.FieldMirroringUtils.toCurrentAllianceTranslation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.GlobalConstants.FieldConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;

public class ShootCommand extends Command {
    private final Shooter shooter;
    private final Swerve swerveDrive;
    /* TODO: Add an option to use AprilTags instead of pose estimation for
    tuning in the shop and potentially as a fallback */
    private final Vision vision;

    private double lastDistanceToHub = -1;
    private double lastShooterMapRPM = -1;

    public ShootCommand(Shooter shooter, Swerve swerveDrive, Vision vision) {
        this.shooter = shooter;
        this.swerveDrive = swerveDrive;
        this.vision = vision;

        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.stopFeeder();
    }

    @Override
    public void execute() {
        /* If we're in the neutral zone or the opposing alliance zone, we're passing
        to our alliance zone. The distance between the hub and the robot will implicitly
        be higher, subsidizing the need to apply the position offset to the hub */

        Translation2d robotTranslation = swerveDrive.getPose().getTranslation();
        Translation2d hubPosition = toCurrentAllianceTranslation(FieldConstants.BLUE_HUB_POSITION);

        // Find distance to the hub and get the corresponding RPM for that distance
        lastDistanceToHub = robotTranslation.getDistance(hubPosition);
        lastShooterMapRPM = DISTANCE_VS_RPM_MAP.get(lastDistanceToHub);

        // Wait until the swerve is aligned to the hub to shoot
        if (Math.abs(swerveDrive.getShootingAngle().getDegrees()) < YAW_ACCEPTABLE_ERROR) {
            shooter.setFlywheelSpeed(RotationsPerMinute.of(lastShooterMapRPM));
        } else {
            shooter.stopShooter();
        }

        // Wait until the shooter's flywheel is up to speed to run the feeder
        if (shooter.shooterIsUpToSpeed()) {
            shooter.setFeederVoltage(Volts.of(FEEDER_VOLTAGE));
        }

        SmartDashboard.putNumber("Distance from hub", lastDistanceToHub);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
    }
}
