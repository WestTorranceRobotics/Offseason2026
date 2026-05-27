package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import org.littletonrobotics.junction.Logger;

public class Module {
    private final ModuleIO io;

    private final SwerveConfigurator.SwerveDriveRobotConstants robotConstants;

    private final PIDController steerPIDController;
    private final PIDController drivePIDController;

    private final SimpleMotorFeedforward driveFeedforward;
    private final double steerVoltageCoefficient;

    private final String moduleName;

    private final SwerveModuleState desiredState = new SwerveModuleState(0, new Rotation2d());
    private final SwerveModuleState currentState = new SwerveModuleState(0, new Rotation2d());

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

        this.drivePIDController =
                new PIDController(moduleConstants.DRIVE_P, moduleConstants.DRIVE_I, moduleConstants.DRIVE_D);
        this.driveFeedforward = new SimpleMotorFeedforward(moduleConstants.DRIVE_S, moduleConstants.DRIVE_V, 0);
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module/" + moduleName, inputs);
    }

    public Rotation2d getSteerAngle() {
        return new Rotation2d(inputs.steerAngleRad);
    }

    public SwerveModuleState getState() {
        currentState.angle = getSteerAngle();
        currentState.speedMetersPerSecond = inputs.driveWheelVelocityRPS * robotConstants.wheelCircumference.in(Meters);
        return currentState;
    }

    public SwerveModulePosition getPosition() {
        swerveModulePosition.angle = getSteerAngle();
        swerveModulePosition.distanceMeters =
                inputs.driveWheelPositionRotations * robotConstants.wheelCircumference.in(Meters);
        return swerveModulePosition;
    }

    public void setDesiredState(LinearVelocity speed, Rotation2d angle) {
        if (angle != null) {
            this.desiredState.angle = angle;
        }
        this.desiredState.speedMetersPerSecond = speed.in(MetersPerSecond);
        this.steerPIDController.setSetpoint(this.desiredState.angle.getRadians());
        this.drivePIDController.setSetpoint(this.desiredState.speedMetersPerSecond);
    }

    public SwerveModuleState getDesiredState() {
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

    // TODO clamp any voltages sent to motors
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

        io.setDriveVoltage(Volts.of(drivePIDController.calculate(
                        inputs.driveWheelVelocityRPS * robotConstants.wheelCircumference.in(Meters))
                + driveFeedforward.calculate(desiredState.speedMetersPerSecond)));
    }
}
