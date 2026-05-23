package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;

    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);
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
