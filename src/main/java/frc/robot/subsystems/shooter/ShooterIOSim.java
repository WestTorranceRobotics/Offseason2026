package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import java.util.Random;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Voltage;

public class ShooterIOSim implements ShooterIO {
    private static final double FLYWHEEL_SHOT_DIP_RPM = new Random().nextInt(500) + 201; // RPM lost per shot (compression)

    private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, 0, MotorType.kBrushless);
    private final SparkMax flywheelMotor = new SparkMax(LAUNCHER_MOTOR_1_ID, 0, MotorType.kBrushless);
    private final SparkMax flywheelMotorInverted = new SparkMax(LAUNCHER_MOTOR_2_ID, 0, MotorType.kBrushless);

    private final SparkMaxSim feederMotorSim;
    private final SparkMaxSim launcherMotorLeaderSim;
    private final SparkMaxSim launcherMotorFollowerSim;

    // TODO: Find MOI (moment of inertia)
    private final FlywheelSim flywheelSim = new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getNEO(2), 0.00062156662, 1), DCMotor.getNEO(2));

    private final FlywheelSim feederSim = new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    private double feederVoltageSetpoint = 0;
    private int lastLaunchedCount = 0;

    public ShooterIOSim() {
        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));
        launcherMotorLeaderSim = new SparkMaxSim(flywheelMotor, DCMotor.getNEO(1));
        launcherMotorFollowerSim = new SparkMaxSim(flywheelMotorInverted, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        updateSim();
        inputs.flywheelRPM = flywheelMotor.getEncoder().getVelocity().get();
        inputs.feederRPM = feederMotor.getEncoder().getVelocity().get();

        checkIfFuelCanBeLaunched(inputs.flywheelRPM);
    }

    private void updateSim() {
        int launched = Robot.fuelSim.getTotalLaunched();
        if (launched > lastLaunchedCount) {
            lastLaunchedCount = launched;
            double dipRadPerSec = FLYWHEEL_SHOT_DIP_RPM * 2.0 * Math.PI / 60.0;
            flywheelSim.setAngularVelocity(Math.max(0, flywheelSim.getAngularVelocity() - dipRadPerSec));
        }

        flywheelSim.setInput(launcherMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        double flywheelRPM = Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocity());
        launcherMotorLeaderSim.iterate(flywheelRPM, RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorFollowerSim.iterate(flywheelRPM, RoboRioSim.getVInVoltage(), 0.02);

        feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        feederSim.update(0.02);
        feederMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(feederSim.getAngularVelocity()),
                RoboRioSim.getVInVoltage(),
                0.02);
    
        double totalCurrent = flywheelSim.getCurrentDraw() + feederSim.getCurrentDraw();
        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(totalCurrent));
    }

    @Override
    public void setFlywheelVoltage(Voltage voltage) {
        flywheelMotor.setVoltage(voltage);
        flywheelMotorInverted.setVoltage(voltage);
    }

    @Override
    public void setFeederVoltage(Voltage voltage) {
        feederMotor.setVoltage(voltage);
        feederVoltageSetpoint = voltage.magnitude();
    }

    private void checkIfFuelCanBeLaunched(double flywheelRPM) {
        boolean feederOn = feederVoltageSetpoint > 0.5;
        if (!feederOn || flywheelRPM < TOLERANCE_TO_RUN_FEEDER) return;

        launchFuel(flywheelRPM);
    }

    private void launchFuel(double flywheelRPM) {
        Pose2d robotPose = RobotContainer.swerveDriveSimulation.getSimulatedDriveTrainPose();
        // The shooter is on the 'back' of the robot, like everything else
        Rotation2d launchHeading = robotPose.getRotation().rotateBy(Rotation2d.k180deg);

        double launchSpeed = flywheelRPM * RPM_TO_EXIT_VELOCITY;
        double vx = launchSpeed * Math.cos(LAUNCH_ANGLE);
        double vy = 0;
        double vz = launchSpeed * Math.sin(LAUNCH_ANGLE);

        Translation3d launchVel = new Translation3d(
                vx * launchHeading.getCos() - vy * launchHeading.getSin(),
                vx * launchHeading.getSin() + vy * launchHeading.getCos(),
                vz);

        // Launcher exit in robot-relative coordinates 
        Translation3d shooterExit = new Translation3d(SHOOTER_X_OFFSET, SHOOTER_Y_OFFSET, SHOOTER_Z_OFFSET);

        // Feed fuel from the hopper through the feeder, then shoot
        Robot.fuelSim.startFeeder(shooterExit, launchVel, flywheelRPM);
    }
}
