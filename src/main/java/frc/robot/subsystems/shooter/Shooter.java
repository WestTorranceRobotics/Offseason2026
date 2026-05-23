package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.ShooterConstants.*;
import static frc.robot.utilities.CustomUnits.RotationsPerMinute;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00242);

    private double targetRPM = 0;
    private double actualRPM = 0;

    public Shooter(ShooterIO io) {
        this.io = io;
        bangbang.setTolerance(0.1);
    }

    @Override
    public void periodic() {
        io.updateInputs();
        SmartDashboard.putNumber("Shooter RPM", actualRPM);
        SmartDashboard.putNumber("Target Shooter RPM", targetRPM);
        SmartDashboard.putNumber("Feeder RPM", io.getFeederRPM());
    }

    public boolean shooterIsUpToSpeed() {
        if (this.targetRPM == 0) {
            return false;
        }
        return Math.abs(this.actualRPM - this.targetRPM) <= TOLERANCE_TO_RUN_FEEDER;
    }

    public Command runShooterCommand(AngularVelocity velocity) {
        return this.runEnd(
                () -> {
                    this.setFlywheelSpeed(velocity);
                    if (this.shooterIsUpToSpeed()) {
                        io.setFeederVoltage(Volts.of(FEEDER_VOLTAGE));
                    }
                },
                this::stopShooter);
    }

    public void setFlywheelSpeed(AngularVelocity velocity) {
        this.targetRPM = velocity.in(RotationsPerMinute);
        bangbang.setSetpoint(targetRPM);
        double voltage = (bangbang.calculate(actualRPM) * RoboRioDataJNI.getVInVoltage())
                + 0.9 * feedforward.calculate(targetRPM);

        io.setFlywheelVoltage(Volts.of(voltage));
    }

    public Command stopShooterCommand() {
        return this.runOnce(this::stopShooter);
    }

    public void stopShooter() {
        this.targetRPM = 0;
        bangbang.setSetpoint(0);
        io.setFlywheelVoltage(Volts.of(0));
        this.stopFeeder();
    }

    public void stopFeeder() {
        io.setFeederVoltage(Volts.of(0));
    }

    public Command runFeederCommand() {
        return this.startEnd(
                () -> {
                    io.setFeederVoltage(Volts.of(FEEDER_VOLTAGE));
                },
                () -> this.stopFeeder());
    }

    public void runFeeder() {
        io.setFeederVoltage(Volts.of(FEEDER_VOLTAGE));
    }

    public Command stopFeederCommand() {
        return this.runOnce(() -> this.stopFeeder());
    }
}
