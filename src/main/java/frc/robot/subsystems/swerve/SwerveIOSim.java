package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;

public class SwerveIOSim implements SwerveDriveIO {
    private final SwerveDriveSimulation3D swerveDriveSimulation;

    public SwerveIOSim() {
        this.swerveDriveSimulation = new SwerveDriveSimulation3D(
                DriveTrainSimulationConfig.Default()
                        .withRobotMass(Pounds.of(75))
                        .withSwerveModule(COTS.ofSwerveX2(
                                edu.wpi.first.math.system.plant.DCMotor.getKrakenX60(1),
                                edu.wpi.first.math.system.plant.DCMotor.getNEO(1),
                                COTS.WHEELS.SLS_PRINTED_WHEELS.cof,
                                2,
                                11))
                        .withTrackLengthTrackWidth(Inches.of(30 - 5), Inches.of(24.5 - 5))
                        .withBumperSize(Inches.of(31), Inches.of(31)),
                new Pose2d(2, 7, Rotation2d.kZero));

        this.swerveDriveSimulation.registerWithArena(Robot.arena, new Pose2d(2, 7, Rotation2d.kZero));
    }

    public SwerveDriveSimulation3D getSimulation() {
        return swerveDriveSimulation;
    }

    @AutoLogOutput(key = "Swerve/Real Pose")
    public Pose3d getSimulationWorldPose() {
        return swerveDriveSimulation.getSimulatedDriveTrainPose3dGroundRelative();
    }

    @Override
    public void setSimulationWorldPose(Pose2d pose) {
        swerveDriveSimulation.setSimulationWorldPose(pose);
    }
}
