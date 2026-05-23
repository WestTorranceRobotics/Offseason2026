package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Module implements ModuleIO {
    private final ModuleIO moduleIO;

    public Module(ModuleIO moduleIO) {
        this.moduleIO = moduleIO;
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        moduleIO.setDriveVoltage(voltage);
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        moduleIO.setSteerVoltage(voltage);
    }

    @Override
    public Voltage getDriveVoltage() {
        return moduleIO.getDriveVoltage();
    }

    @Override
    public Voltage getSteerVoltage() {
        return moduleIO.getSteerVoltage();
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return moduleIO.getSteerVelocity();
    }

    @Override
    public Rotation2d getSteerAngle() {
        return moduleIO.getSteerAngle();
    }

    @Override
    public Angle getDriveWheelPosition() {
        return moduleIO.getDriveWheelPosition();
    }

    @Override
    public AngularVelocity getDriveWheelVelocity() {
        return moduleIO.getDriveWheelVelocity();
    }

    @Override
    public SwerveModuleState getState() {
        return moduleIO.getState();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return moduleIO.getPosition();
    }

    @Override
    public void setSteerPID(double angle) {
        moduleIO.setSteerPID(angle);
    }

    @Override
    public void setDrivePID(double speed) {
        moduleIO.setDrivePID(speed);
    }

    @Override
    public void setDesiredState(LinearVelocity speed, Rotation2d angle) {
        moduleIO.setDesiredState(speed, angle);
    }

    @Override
    public SwerveModuleState getDesiredState() {
        return moduleIO.getDesiredState();
    }

    @Override
    public void tickPID() {
        moduleIO.tickPID();
    }

    @Override
    public String getModuleName() {
        return moduleIO.getModuleName();
    }

    @Override
    public Translation2d getUnitRotationVec() {
        return moduleIO.getUnitRotationVec();
    }

    @Override
    public void updateInputs() {
        moduleIO.updateInputs();
    }

    @Override
    public void telemetryHook(edu.wpi.first.util.sendable.SendableBuilder sendableBuilder) {
        moduleIO.telemetryHook(sendableBuilder);
    }
}
