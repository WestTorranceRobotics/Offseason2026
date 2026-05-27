package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleIOSim implements ModuleIO {
    private final SwerveModuleSimulation swerveModuleSimulation;
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController steerMotor;

    public ModuleIOSim(SwerveModuleSimulation swerveModuleSimulation) {
        this.swerveModuleSimulation = swerveModuleSimulation;
        this.driveMotor =
                swerveModuleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(80));
        this.steerMotor = swerveModuleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
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
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveMotor.requestVoltage(voltage);
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        steerMotor.requestVoltage(voltage);
    }
}
