package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double intakeRPM = 0.0;
        public double pivotRPM = 0.0;

        public String pivotPosition = "UP";
    }

    public void setIntakeVoltage(Voltage voltage);

    public void setPivotVoltage(Voltage voltage);

    public default void updateInputs(IntakeIOInputs inputs) {}
}
