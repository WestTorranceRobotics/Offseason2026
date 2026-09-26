package frc.robot.subsystems.swerve.module;

import static org.wpilib.units.Units.*;

import frc.robot.subsystems.swerve.SwerveConfigurator;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation swerveModuleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController steerMotor;

    // Software closed-loop for the simulated drive motor; gains operate in meters/second
    private final PIDController drivePIDController;
    private final SimpleMotorFeedforward driveFeedforward;
    private final Distance wheelCircumference;

    public ModuleIOSim(
            SwerveModuleSimulation swerveModuleSimulation,
            SwerveConfigurator.SwerveDriveRobotConstants robotConstants,
            SwerveConfigurator.SwerveDriveModuleConstants moduleConstants) {
        this.swerveModuleSimulation = swerveModuleSimulation;
        this.driveMotor =
                swerveModuleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(80));
        this.steerMotor = swerveModuleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));

        this.wheelCircumference = robotConstants.wheelCircumference;
        this.drivePIDController =
                new PIDController(moduleConstants.DRIVE_P, moduleConstants.DRIVE_I, moduleConstants.DRIVE_D);
        this.driveFeedforward = new SimpleMotorFeedforward(moduleConstants.DRIVE_S, moduleConstants.DRIVE_V, 0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVoltage = driveMotor.getAppliedVoltage().magnitude();
        inputs.steerVoltage = steerMotor.getAppliedVoltage().magnitude();

        inputs.driveWheelPositionRotations =
                swerveModuleSimulation.getDriveWheelFinalPosition().in(Rotations);
        inputs.driveWheelVelocityRPS =
                swerveModuleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond);

        inputs.steerAngleRad = swerveModuleSimulation.getSteerAbsoluteFacing().getRadians();
        inputs.steerVelocityRadPerSec =
                swerveModuleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);

        inputs.driveCurrent =
                swerveModuleSimulation.getDriveMotorSupplyCurrent().in(Amps);
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveMotor.requestVoltage(voltage);
    }

    @Override
    public void setDriveVelocity(AngularVelocity wheelVelocity) {
        double desiredMetersPerSecond = wheelVelocity.in(RotationsPerSecond) * wheelCircumference.in(Meters);
        double currentMetersPerSecond =
                swerveModuleSimulation.getDriveWheelFinalSpeed().in(RotationsPerSecond) * wheelCircumference.in(Meters);
        driveMotor.requestVoltage(Volts.of(drivePIDController.calculate(currentMetersPerSecond, desiredMetersPerSecond)
                + driveFeedforward.calculate(desiredMetersPerSecond)));
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        steerMotor.requestVoltage(voltage);
    }
}
