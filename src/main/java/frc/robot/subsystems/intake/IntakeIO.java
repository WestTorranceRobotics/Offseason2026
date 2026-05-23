package frc.robot.subsystems.intake;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface IntakeIO {
    public double getIntakeRPM();

    public double getHoodRPM();

    public void setIntakeVoltage(Voltage voltage);

    public void setHoodVoltage(Voltage voltage);

    public String getIntakeLocation();

    public default void updateInputs() {}
}
