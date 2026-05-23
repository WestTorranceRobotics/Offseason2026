package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static org.ironmaple.utils.FieldMirroringUtils.isSidePresentedAsRed;
import static org.ironmaple.utils.FieldMirroringUtils.toCurrentAllianceTranslation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GlobalConstants.FieldConstants;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;
import frc.robot.subsystems.swerve.gyro.Gyro;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.module.Module;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;

public class Swerve extends SubsystemBase {
    private final Gyro gyro;
    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    private final Module frontLeft;
    private final Module frontRight;
    private final Module backLeft;
    private final Module backRight;

    public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.3175, 0.24765), // Front left
            new Translation2d(0.3175, -0.24765), // Front right
            new Translation2d(-0.3175, 0.24765), // Back left
            new Translation2d(-0.3175, -0.24765)); // Back right

    private final SwerveDriveIO io;

    public Swerve(SwerveDriveIO io, GyroIO gyroIO, Module[] modules) {
        this.io = io;
        this.gyro = new Gyro(gyroIO);

        this.frontLeft = modules[0];
        this.frontRight = modules[1];
        this.backLeft = modules[2];
        this.backRight = modules[3];

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics, gyro.getRotation(), getModulePositions(), Pose2d.kZero);

        RobotConfig ppConfig;
        try {
            ppConfig = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            throw new RuntimeException("Failed to get PathPlanner config from GUI");
        }

        AutoBuilder.configure(
                this::getPose,
                this::setPose,
                this::getChassisSpeed,
                (ChassisSpeeds speeds) -> drive(speeds, false),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                RealRobotConstants.TRANSLATION_P,
                                RealRobotConstants.TRANSLATION_I,
                                RealRobotConstants.TRANSLATION_D),
                        new PIDConstants(
                                RealRobotConstants.ROTATION_P,
                                RealRobotConstants.ROTATION_I,
                                RealRobotConstants.ROTATION_D)),
                ppConfig,
                () -> {
                    return isSidePresentedAsRed();
                });
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (fieldRelative) {
            Rotation2d fieldRelativeHeading =
                    Rotation2d.fromRadians(getHeading().getRadians() + (isSidePresentedAsRed() ? Math.PI : 0));
            Rotation2d skewCompensation = Rotation2d.fromRadians(
                    chassisSpeeds.omegaRadiansPerSecond * SwerveDriveConstants.SKEW_COMPENSATION_FACTOR);
            chassisSpeeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, fieldRelativeHeading.plus(skewCompensation));
        }
        calculateStates(chassisSpeeds);
    }

    public Rotation2d getShootingAngle() {
        Translation2d robotTranslation = getPose().getTranslation();
        Translation2d hubPosition = toCurrentAllianceTranslation(FieldConstants.BLUE_HUB_POSITION);

        // Detect if we aren't in our alliance zone
        if ((hubPosition.getX() - robotTranslation.getX()) * (isSidePresentedAsRed() ? -1 : 1) < 0) {
            // Applies an offset to align to the closest corner of our alliance zone to pass to
            hubPosition = hubPosition.plus(new Translation2d(
                    0,
                    Math.signum(robotTranslation.getY() - hubPosition.getY())
                            * SwerveDriveConstants.PASS_OFFSET_FACTOR));
        }

        return new Rotation2d(Math.atan2(
                        hubPosition.getY() - robotTranslation.getY(), hubPosition.getX() - robotTranslation.getX()))
                .minus(getHeading())
                .rotateBy(Rotation2d.k180deg);
    }

    @Override
    public void periodic() {
        gyro.updateInputs();

        frontLeft.updateInputs();
        frontRight.updateInputs();
        backLeft.updateInputs();
        backRight.updateInputs();

        io.updateInputs();

        swerveDrivePoseEstimator.update(gyro.getRotation(), getModulePositions());
    }

    private void calculateStates(ChassisSpeeds chassisSpeeds) {
        Module[] modules = getModules();

        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < modules.length; i++) {
            moduleStates[i].optimize(modules[i].getSteerAngle());
            moduleStates[i].speedMetersPerSecond *=
                    moduleStates[i].angle.minus(modules[i].getSteerAngle()).getCos();

            modules[i].setDesiredState(MetersPerSecond.of(moduleStates[i].speedMetersPerSecond), moduleStates[i].angle);
            modules[i].tickPID();
        }
    }

    public ChassisSpeeds getChassisSpeed() {
        ChassisSpeeds chassisSpeeds = swerveDriveKinematics.toChassisSpeeds(getModuleStates());
        return chassisSpeeds;
    }

    @AutoLogOutput(key = "Swerve/Odometry")
    public Pose2d getPose() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        io.setSimulationWorldPose(pose);
        swerveDrivePoseEstimator.resetPosition(gyro.getRotation(), getModulePositions(), pose);
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        swerveDrivePoseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        swerveDrivePoseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(), backLeft.getPosition(), backRight.getPosition()
        };
    }

    @AutoLogOutput(key = "Swerve/Current Module States")
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
        };
    }

    @AutoLogOutput(key = "Swerve/Desired Module States")
    public SwerveModuleState[] getDesiredModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getDesiredState(),
            frontRight.getDesiredState(),
            backLeft.getDesiredState(),
            backRight.getDesiredState()
        };
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d rotation2d) {
        setPose(new Pose2d(getPose().getTranslation(), rotation2d));
    }

    public void zeroHeading() {
        swerveDrivePoseEstimator.resetPosition(
                gyro.getRotation(), getModulePositions(), new Pose2d(getPose().getTranslation(), Rotation2d.kZero));
    }

    public Module[] getModules() {
        return new Module[] {frontLeft, frontRight, backLeft, backRight};
    }

    public Command lockWheelsInX() {
        return Commands.run(() -> {
            Module[] modules = getModules();
            for (int i = 0; i < 4; i++) {
                modules[i].setDesiredState(MetersPerSecond.zero(), Rotation2d.fromDegrees((i * 90) - 45));
            }
        });
    }
}
