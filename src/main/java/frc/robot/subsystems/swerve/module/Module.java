package frc.robot.subsystems.swerve.module;

import static org.wpilib.units.Units.*;

import frc.robot.subsystems.swerve.SwerveConfigurator;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

public class Module {
    private final ModuleIO io;

    private final SwerveConfigurator.SwerveDriveRobotConstants robotConstants;

    private final PIDController steerPIDController;

    private final double steerVoltageCoefficient;

    private final String moduleName;

    private final SwerveModuleVelocity desiredState = new SwerveModuleVelocity(0, new Rotation2d());
    private final SwerveModuleVelocity currentState = new SwerveModuleVelocity(0, new Rotation2d());

    private final SwerveModulePosition swerveModulePosition = new SwerveModulePosition();

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    public Module(
            ModuleIO io,
            SwerveConfigurator.SwerveDriveRobotConstants robotConstants,
            SwerveConfigurator.SwerveDriveModuleConstants moduleConstants) {
        this.io = io;
        this.robotConstants = robotConstants;

        this.moduleName = moduleConstants.getModuleName();

        this.steerVoltageCoefficient = moduleConstants.azimuthReversed ? -1 : 1;
        this.steerPIDController =
                new PIDController(moduleConstants.AZIMUTH_P, moduleConstants.AZIMUTH_I, moduleConstants.AZIMUTH_D);
        this.steerPIDController.enableContinuousInput(-Math.PI, Math.PI);
        this.steerPIDController.setTolerance(0.05);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module/" + moduleName, inputs);
    }

    public Rotation2d getSteerAngle() {
        return new Rotation2d(inputs.steerAngleRad);
    }

    public SwerveModuleVelocity getState() {
        currentState.angle = getSteerAngle();
        currentState.velocity = inputs.driveWheelVelocityRPS * robotConstants.wheelCircumference.in(Meters);
        return currentState;
    }

    public SwerveModulePosition getPosition() {
        swerveModulePosition.angle = getSteerAngle();
        swerveModulePosition.distance =
                inputs.driveWheelPositionRotations * robotConstants.wheelCircumference.in(Meters);
        return swerveModulePosition;
    }

    public void setDesiredState(LinearVelocity speed, Rotation2d angle) {
        if (angle != null) {
            desiredState.angle = angle;
        }
        desiredState.velocity = speed.in(MetersPerSecond);
        steerPIDController.setSetpoint(desiredState.angle.getRadians());
    }

    public SwerveModuleVelocity getDesiredState() {
        return desiredState;
    }

    public void setDriveVoltage(Voltage voltage) {
        io.setDriveVoltage(voltage);
    }

    public void setSteerVoltage(Voltage voltage) {
        io.setSteerVoltage(voltage);
    }

    public void setSteerPID(double angle) {
        steerPIDController.setSetpoint(angle);
    }

    public AngularVelocity getDriveWheelVelocity() {
        return RotationsPerSecond.of(inputs.driveWheelVelocityRPS);
    }

    public Angle getDriveWheelPosition() {
        return Rotations.of(inputs.driveWheelPositionRotations);
    }

    // TODO: clamp any voltages sent to motors
    public Voltage getDriveVoltage() {
        return Volts.of(inputs.driveVoltage);
    }

    public Voltage getSteerVoltage() {
        return Volts.of(inputs.steerVoltage);
    }

    public AngularVelocity getSteerVelocity() {
        return RadiansPerSecond.of(inputs.steerVelocityRadPerSec);
    }

    public void tickPID() {
        io.setSteerVoltage(Volts.of(steerVoltageCoefficient * steerPIDController.calculate(inputs.steerAngleRad)));

        // Drive closed-loop control is separated
        io.setDriveVelocity(
                RotationsPerSecond.of(desiredState.velocity / robotConstants.wheelCircumference.in(Meters)));
    }
}
