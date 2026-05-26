package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants;
import frc.robot.constants.SwerveDriveConstants.RealRobotConstants.RealModuleConstants;
import frc.robot.constants.SwerveDriveConstants.SimulatedControlSystemConstants.SimulatedModuleConstants;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIOReal;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import java.util.HashSet;
import java.util.Set;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;

public class SwerveConfigurator {
    public final SwerveDriveRobotConstants swerveDriveRobotConstants;

    private final SwerveDriveModuleConstants[] moduleConstants;

    public SwerveConfigurator(
            SwerveDriveRobotConstants swerveDriveRobotConstants, SwerveDriveModuleConstants[] moduleConstants) {
        this.swerveDriveRobotConstants = swerveDriveRobotConstants;

        Set<SwerveModuleCornerPosition> swerveModuleCornerPositionSet = new HashSet<>();
        this.moduleConstants = moduleConstants;

        for (SwerveDriveModuleConstants moduleConstant : moduleConstants) {

            swerveModuleCornerPositionSet.add(moduleConstant.corner);

            moduleConstant.calculateModulePosition(
                    swerveDriveRobotConstants.driveBaseWidth,
                    swerveDriveRobotConstants.driveBaseLength,
                    swerveDriveRobotConstants.moduleDistanceFromEdge);
        }

        if (moduleConstants.length < 4) {
            DriverStation.reportWarning("Less than four modules were defined in the configurator", false);
        }

        if (moduleConstants.length > 4) {
            DriverStation.reportError("More than five modules were defined in the configurator", false);
        }

        if (swerveModuleCornerPositionSet.size() != moduleConstants.length) {
            DriverStation.reportError("Duplicate swerve module positions were configured", false);
        }
    }

    public SwerveDriveModuleConstants getModuleConstants(SwerveModuleCornerPosition corner) {
        for (SwerveDriveModuleConstants moduleConstant : moduleConstants) {
            if (moduleConstant.corner == corner) {
                return moduleConstant;
            }
        }

        throw new RuntimeException("Tried to get configuration of an unconfigured module");
    }

    public static SwerveConfigurator defaultRealConfigurator() {
        return new SwerveConfigurator(
                new SwerveDriveRobotConstants(
                        Kilograms.of(35),
                        Inches.of(30),
                        Inches.of(24.5),
                        Inches.of(2.5),
                        Inches.of(2),
                        RealRobotConstants.PIGEON2_ID),
                new SwerveDriveModuleConstants[] {
                    RealModuleConstants.FLModuleConstants,
                    RealModuleConstants.FRModuleConstants,
                    RealModuleConstants.BLModuleConstants,
                    RealModuleConstants.BRModuleConstants
                });
    }

    public static SwerveConfigurator defaultSimConfigurator() {
        return new SwerveConfigurator(
                new SwerveDriveRobotConstants(
                        Pounds.of(75), Inches.of(30), Inches.of(24.5), Inches.of(2.5), Inches.of(2), 0),
                new SwerveDriveModuleConstants[] {
                    SimulatedModuleConstants.FLModuleConstants,
                    SimulatedModuleConstants.FRModuleConstants,
                    SimulatedModuleConstants.BLModuleConstants,
                    SimulatedModuleConstants.BRModuleConstants
                });
    }

    public static Module[] createRealModules() {
        SwerveConfigurator defaultConfig = defaultRealConfigurator();
        // It might be too redundant to have to define the module position twice
        return new Module[] {
            new Module(
                    new ModuleIOReal(SwerveModuleCornerPosition.FRONT_LEFT, defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.FRONT_LEFT)),
            new Module(
                    new ModuleIOReal(SwerveModuleCornerPosition.FRONT_RIGHT, defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.FRONT_RIGHT)),
            new Module(
                    new ModuleIOReal(SwerveModuleCornerPosition.BACK_LEFT, defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.BACK_LEFT)),
            new Module(
                    new ModuleIOReal(SwerveModuleCornerPosition.BACK_RIGHT, defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.BACK_RIGHT))
        };
    }

    public static Module[] createSimModules(SwerveDriveSimulation3D swerveDriveSimulation) {
        SwerveConfigurator defaultConfig = defaultSimConfigurator();
        // It might be too redundant to have to define the module position twice
        return new Module[] {
            new Module(
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[0],
                            SwerveModuleCornerPosition.FRONT_LEFT,
                            defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.FRONT_LEFT)),
            new Module(
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[1],
                            SwerveModuleCornerPosition.FRONT_RIGHT,
                            defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.FRONT_RIGHT)),
            new Module(
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[2], SwerveModuleCornerPosition.BACK_LEFT, defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.BACK_LEFT)),
            new Module(
                    new ModuleIOSim(
                            swerveDriveSimulation.getModules()[3],
                            SwerveModuleCornerPosition.BACK_RIGHT,
                            defaultConfig),
                    defaultConfig.swerveDriveRobotConstants,
                    defaultConfig.getModuleConstants(SwerveModuleCornerPosition.BACK_RIGHT))
        };
    }

    public static class SwerveDriveRobotConstants {
        public final Mass robotMass;
        public final Distance driveBaseLength;
        public final Distance driveBaseWidth;
        public final Distance moduleDistanceFromEdge;
        public final Distance wheelRadius;
        public final Distance wheelCircumference;

        public final int pigeonID;

        public SwerveDriveRobotConstants(
                Mass robotMass,
                Distance driveBaseLength,
                Distance driveBaseWidth,
                Distance moduleDistanceFromEdge,
                Distance wheelRadius,
                int pigeonID) {
            this.robotMass = robotMass;
            this.driveBaseLength = driveBaseLength;
            this.driveBaseWidth = driveBaseWidth;
            this.moduleDistanceFromEdge = moduleDistanceFromEdge;
            this.wheelRadius = wheelRadius;
            this.wheelCircumference = wheelRadius.times(2 * Math.PI);

            this.pigeonID = pigeonID;
        }
    }

    public static class SwerveDriveModuleConstants {
        private final SwerveModuleCornerPosition corner;

        public final int CANCoderID;
        public final int driveMotorID;
        public final int azimuthMotorID;

        public final double CANCoderOffset;

        public final double DRIVE_P;
        public final double DRIVE_I;
        public final double DRIVE_D;
        public final double DRIVE_S;
        public final double DRIVE_V;

        public final double AZIMUTH_P;
        public final double AZIMUTH_I;
        public final double AZIMUTH_D;
        public final double AZIMUTH_S;

        public final double driveGearRatio;

        public final boolean azimuthReversed;

        public Translation2d physicalModulePosition;

        public SwerveDriveModuleConstants(
                SwerveDriveModuleConstants clonedConstants,
                SwerveModuleCornerPosition corner,
                int CANCoderID,
                int driveMotorID,
                int azimuthMotorID,
                double CANCoderOffset) {
            this.corner = corner;

            this.CANCoderID = CANCoderID;
            this.driveMotorID = driveMotorID;
            this.azimuthMotorID = azimuthMotorID;

            this.CANCoderOffset = CANCoderOffset;

            this.DRIVE_P = clonedConstants.DRIVE_P;
            this.DRIVE_I = clonedConstants.DRIVE_I;
            this.DRIVE_D = clonedConstants.DRIVE_D;
            this.DRIVE_S = clonedConstants.DRIVE_S;
            this.DRIVE_V = clonedConstants.DRIVE_V;

            this.AZIMUTH_P = clonedConstants.AZIMUTH_P;
            this.AZIMUTH_I = clonedConstants.AZIMUTH_I;
            this.AZIMUTH_D = clonedConstants.AZIMUTH_D;
            this.AZIMUTH_S = clonedConstants.AZIMUTH_S;

            this.driveGearRatio = clonedConstants.driveGearRatio;

            this.azimuthReversed = clonedConstants.azimuthReversed;
        }

        // TODO fix wheel spinning when wheel is rotating (like planetary gears)
        public SwerveDriveModuleConstants(
                SwerveModuleCornerPosition corner,
                int CANCoderID,
                int driveMotorID,
                int azimuthMotorID,
                double CANCoderOffset,
                double DRIVE_P,
                double DRIVE_I,
                double DRIVE_D,
                double DRIVE_S,
                double DRIVE_V,
                double AZIMUTH_P,
                double AZIMUTH_I,
                double AZIMUTH_D,
                double AZIMUTH_S,
                int azimuthReverseCount,
                boolean shaftFacingUp,
                double driveGearRatio) {
            this.corner = corner;

            this.CANCoderID = CANCoderID;
            this.driveMotorID = driveMotorID;
            this.azimuthMotorID = azimuthMotorID;

            this.CANCoderOffset = CANCoderOffset;

            this.DRIVE_P = DRIVE_P;
            this.DRIVE_I = DRIVE_I;
            this.DRIVE_D = DRIVE_D;
            this.DRIVE_S = DRIVE_S;
            this.DRIVE_V = DRIVE_V;

            this.AZIMUTH_P = AZIMUTH_P;
            this.AZIMUTH_I = AZIMUTH_I;
            this.AZIMUTH_D = AZIMUTH_D;
            this.AZIMUTH_S = AZIMUTH_S;

            this.driveGearRatio = driveGearRatio;

            //   TODO check NEO/SparkMAX defaults to ccw+

            boolean reversed = !shaftFacingUp;
            if (azimuthReverseCount % 2 == 0) reversed = !reversed;

            azimuthReversed = reversed;
        }

        private void calculateModulePosition(Distance baseWidth, Distance baseLength, Distance moduleDistanceFromEdge) {
            double x = baseWidth.div(2).in(Meters);
            double y = baseLength.div(2).in(Meters);

            double moduleDist = moduleDistanceFromEdge.in(Meters);

            this.physicalModulePosition = new Translation2d(x, y)
                    .minus(new Translation2d(moduleDist, moduleDist))
                    .rotateBy(Rotation2d.kCCW_90deg.times(corner.index));
        }

        public String getModuleName() {
            return corner.getName();
        }
    }

    public enum SwerveModuleCornerPosition {
        FRONT_RIGHT("Front Right", 0),

        FRONT_LEFT("Front Left", 1),

        BACK_LEFT("Back Left", 2),

        BACK_RIGHT("Back Right", 3);

        private final String name;
        private final int index;

        SwerveModuleCornerPosition(String name, int id) {
            this.name = name;
            this.index = id;
        }

        public String getName() {
            return name;
        }

        public int getIndex() {
            return index;
        }
    }
}
