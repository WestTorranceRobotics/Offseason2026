package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Voltage;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double intakeRPM = 0.0;
        public double pivotRPM = 0.0;

        public double intakeCurrent = 0.0;

        public String pivotPosition = "UP";
    }

    public void setIntakeVoltage(Voltage voltage);

    public void setPivotVoltage(Voltage voltage);

    public default void updateInputs(IntakeIOInputs inputs) {}
}
