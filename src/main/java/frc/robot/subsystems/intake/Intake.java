package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Intake extends SubsystemBase {
    private final IntakeIO io;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs();
        SmartDashboard.putNumber("Intake RPM", getIntakeRPM());
        SmartDashboard.putNumber("Hood RPM", getHoodRPM());
    }

    public double getIntakeRPM() {
        return io.getIntakeRPM();
    }

    public double getHoodRPM() {
        return io.getHoodRPM();
    }

    public Command intakeCommand() {
        return this.startEnd(this::intake, this::stopIntake);
    }

    public Command outtakeCommand() {
        return this.startEnd(this::outtake, this::stopIntake);
    }

    public Command stopIntakeCommand() {
        return this.runOnce(this::stopIntake);
    }

    public void outtake() {
        io.setIntakeVoltage(Volts.of(OUTTAKE_VOLTAGE));
    }

    public void intake() {
        io.setIntakeVoltage(Volts.of(INTAKE_VOLTAGE));
    }

    public void setIntakeVoltage(Voltage voltage) {
        io.setIntakeVoltage(voltage);
    }

    public void stopIntake() {
        io.setIntakeVoltage(Volts.of(0));
    }

    public Command sendHoodDownCommand() {
        return this.runHoodAtVoltageCommand(Volts.of(PIVOT_DOWN_VOLTAGE));
    }

    public Command sendHoodUpCommand() {
        return this.runHoodAtVoltageCommand(Volts.of(PIVOT_UP_VOLTAGE));
    }

    public Command runHoodAtVoltageCommand(Voltage voltage) {
        return this.runEnd(() -> io.setHoodVoltage(voltage), () -> io.setHoodVoltage(Volts.of(0)));
    }

    public Command stopHoodCommand() {
        return this.runOnce(() -> io.setHoodVoltage(Volts.of(0)));
    }
}
