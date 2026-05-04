package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        SmartDashboard.putNumber("Intake RPM", io.getIntakeRPM());
        SmartDashboard.putNumber("Pivot RPM", io.getPivotRPM());
    }

    public void setIntakeVoltage(Voltage voltage) {
        io.setIntakeVoltage(voltage);
    }

    public void stopIntake() {
        io.setIntakeVoltage(Volts.of(0));
    }

    public void setPivotVoltage(Voltage voltage) {
        io.setPivotVoltage(voltage);
    }

    public void stopPivot() {
        io.setPivotVoltage(Volts.of(0));
    }
}
