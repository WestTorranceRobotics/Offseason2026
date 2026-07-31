package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;
import static org.wpilib.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.hal.simulation.RoboRioDataJNI;
import org.wpilib.math.controller.BangBangController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;

    private final BangBangController bangbang = new BangBangController();
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.00242);

    @AutoLogOutput
    private double targetFlywheelRPM = 0;

    private double flywheelRPM = 0;

    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    public Shooter(ShooterIO io) {
        this.io = io;
        bangbang.setTolerance(0.1);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);
        flywheelRPM = inputs.flywheelRPM;
    }

    public boolean shooterIsUpToSpeed() {
        if (this.targetFlywheelRPM == 0) {
            return false;
        }
        return Math.abs(this.flywheelRPM - this.targetFlywheelRPM) <= TOLERANCE_TO_RUN_FEEDER;
    }

    public void setFlywheelSpeed(AngularVelocity velocity) {
        this.targetFlywheelRPM = velocity.in(RPM);
        bangbang.setSetpoint(targetFlywheelRPM);
        double voltage = (bangbang.calculate(flywheelRPM) * RoboRioDataJNI.getVInVoltage())
                + 0.9 * feedforward.calculate(targetFlywheelRPM);

        io.setFlywheelVoltage(Volts.of(voltage));
    }

    public void stopShooter() {
        this.targetFlywheelRPM = 0;
        bangbang.setSetpoint(0);
        io.setFlywheelVoltage(Volts.of(0));
        this.stopFeeder();
    }

    public void stopFeeder() {
        io.setFeederVoltage(Volts.of(0));
    }

    public void setFeederVoltage(Voltage voltage) {
        io.setFeederVoltage(voltage);
    }

    public double getFeederRPM() {
        return inputs.feederRPM;
    }
}
