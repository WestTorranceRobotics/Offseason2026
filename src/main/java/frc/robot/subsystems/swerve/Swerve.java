package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.GlobalConstants.isAllianceBlue;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.epilogue.Logged;
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
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;
import frc.robot.subsystems.swerve.gyroscope.Gyro;
import frc.robot.subsystems.swerve.gyroscope.GyroIO;
import frc.robot.subsystems.swerve.module.Module;
import java.io.IOException;
import org.json.simple.parser.ParseException;

@Logged
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

    private final StructPublisher<Pose2d> estimatedPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Estimated Pose", Pose2d.struct)
            .publish();
    private final StructArrayPublisher<SwerveModuleState> currentModuleStatesPublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Current Module States", SwerveModuleState.struct)
                    .publish();
    private final StructArrayPublisher<SwerveModuleState> desiredModuleStatesPublisher =
            NetworkTableInstance.getDefault()
                    .getStructArrayTopic("Desired Module States", SwerveModuleState.struct)
                    .publish();

    // These are used to override driver rotation if aligning
    private boolean isAligning;
    private double targetRotationYaw;

    private final SwerveDriveIO io;

    public Swerve(SwerveDriveIO io, GyroIO gyroIO, Module[] modules) {
        this.io = io;
        this.gyro = new Gyro(gyroIO);

        this.frontLeft = modules[0];
        this.frontRight = modules[1];
        this.backLeft = modules[2];
        this.backRight = modules[3];

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                swerveDriveKinematics, this.gyro.getRotation(), getModulePositions(), Pose2d.kZero);

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
                (ChassisSpeeds speeds) -> this.drive(speeds, false),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                RealRobotConstants.kPTranslation,
                                RealRobotConstants.kITranslation,
                                RealRobotConstants.kDTranslation),
                        new PIDConstants(
                                RealRobotConstants.kPRotation,
                                RealRobotConstants.kIRotation,
                                RealRobotConstants.kDRotation)),
                ppConfig,
                () -> {
                    if (isAllianceBlue()) {
                        return false;
                    }
                    return true;
                });

        initTelemetry();
    }

    public void drive(ChassisSpeeds chassisSpeeds, boolean fieldRelative) {
        if (isAligning) {
            chassisSpeeds = createAlignChassisSpeeds(
                    chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, targetRotationYaw);
        } else if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    chassisSpeeds,
                    Rotation2d.fromRadians(this.getHeading().getRadians() + (isAllianceBlue() ? 0 : Math.PI))
                            .plus(Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond
                                    * SwerveDriveConstants.SKEW_COMPENSATION_FACTOR)));
        }
        calculateStates(chassisSpeeds);
    }

    public ChassisSpeeds createAlignChassisSpeeds(
            double vxMetersPerSecond, double vyMetersPerSecond, double targetRotationYaw) {
        return new ChassisSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                -targetRotationYaw
                        * SwerveDriveConstants.PROPORTIONALITY_CONSTANT
                        * SwerveDriveConstants.MAX_ANGULAR_SPEED);
    }

    public void setAlignStatus(boolean isAligning, double targetRotationYaw) {
        this.isAligning = isAligning;
        this.targetRotationYaw = targetRotationYaw;
    }

    public Rotation2d getAngleToHub() {
        Translation2d robotTranslation = getPose().getTranslation();
        Translation2d hubPosition = isAllianceBlue()
                ? GlobalConstants.FieldConstants.BLUE_HUB_POSITION
                : GlobalConstants.FieldConstants.RED_HUB_POSITION;

        return new Rotation2d(
                Math.atan2(hubPosition.getY() - robotTranslation.getY(), hubPosition.getX() - robotTranslation.getX()));
    }

    @Override
    public void periodic() {
        gyro.updateInputs();
        for (Module module : getModules()) {
            module.updateInputs();
        }

        io.updateInputs();
        swerveDrivePoseEstimator.update(gyro.getRotation(), getModulePositions());

        publishTelemetry();
    }

    public void tickPid() {
        // Some PIDs need to be calculated, do that here
        frontRight.tickPID();
        frontLeft.tickPID();
        backRight.tickPID();
        backLeft.tickPID();
    }

    private void publishTelemetry() {
        estimatedPosePublisher.set(swerveDrivePoseEstimator.getEstimatedPosition());
        currentModuleStatesPublisher.set(getModuleStates());
        desiredModuleStatesPublisher.set(new SwerveModuleState[] {
            frontLeft.getDesiredState(),
            frontRight.getDesiredState(),
            backLeft.getDesiredState(),
            backRight.getDesiredState()
        });
    }

    private void calculateStates(ChassisSpeeds chassisSpeeds) {
        Module[] modules = this.getModules();

        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);
        SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        for (int i = 0; i < modules.length; i++) {
            moduleStates[i].optimize(modules[i].getSteerAngle());
            moduleStates[i].speedMetersPerSecond *=
                    moduleStates[i].angle.minus(modules[i].getSteerAngle()).getCos();

            modules[i].setDesiredState(MetersPerSecond.of(moduleStates[i].speedMetersPerSecond), moduleStates[i].angle);
        }
    }

    public ChassisSpeeds getChassisSpeed() {
        ChassisSpeeds chassisSpeeds = swerveDriveKinematics.toChassisSpeeds(getModuleStates());
        return chassisSpeeds;
    }

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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()
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

    public void initTelemetry() {
        SmartDashboard.putData("SwerveDriveTelemetry", (builder) -> {
            builder.setSmartDashboardType("SwerveDriveTelemetry");
        });
    }

    public Module[] getModules() {
        return new Module[] {frontLeft, frontRight, backLeft, backRight};
    }

    public Command lockWheelsInX() {
        return Commands.run(() -> {
            this.setAlignStatus(false, 0);
            var modules = this.getModules();
            for (int i = 0; i < 4; i++) {
                modules[i].setDesiredState(MetersPerSecond.zero(), Rotation2d.fromDegrees((i * 90) - 45));
            }
        });
    }
}
