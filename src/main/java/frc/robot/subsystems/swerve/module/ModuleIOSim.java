package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation swerveModuleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController steerMotor;

    public ModuleIOSim(
            SwerveModuleSimulation swerveModuleSimulation,
            SwerveConfigurator.SwerveModuleCornerPosition cornerPosition,
            SwerveConfigurator swerveDriveConfigurator) {
        this.swerveModuleSimulation = swerveModuleSimulation;
        this.driveMotor =
                swerveModuleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(80));
        this.steerMotor = swerveModuleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVoltage = getDriveVoltage().magnitude();
        inputs.steerVoltage = getSteerVoltage().magnitude();
        inputs.driveWheelPositionRotations = getDriveWheelPosition().in(Rotations);
        inputs.driveWheelVelocityRPS = getDriveWheelVelocity().in(RotationsPerSecond);
        inputs.steerAngleRad = getSteerAngle().getRadians();
        inputs.steerVelocityRadPerSec = getSteerVelocity().in(RadiansPerSecond);
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        this.driveMotor.requestVoltage(voltage);
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        this.steerMotor.requestVoltage(voltage);
    }

    @Override
    public Voltage getDriveVoltage() {
        return this.driveMotor.getAppliedVoltage();
    }

    @Override
    public Voltage getSteerVoltage() {
        return this.steerMotor.getAppliedVoltage();
    }

    @Override
    public Rotation2d getSteerAngle() {
        return swerveModuleSimulation.getSteerAbsoluteFacing();
    }

    @Override
    public Angle getDriveWheelPosition() {
        return swerveModuleSimulation.getDriveWheelFinalPosition();
    }

    @Override
    public AngularVelocity getDriveWheelVelocity() {
        return swerveModuleSimulation.getDriveWheelFinalSpeed();
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return swerveModuleSimulation.getSteerAbsoluteEncoderSpeed();
    }
}
