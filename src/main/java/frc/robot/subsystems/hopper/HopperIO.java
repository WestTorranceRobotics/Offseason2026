package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;

public interface HopperIO {
    public double getRollerRPM();

    public void setRollerVoltage(Voltage voltage);

    public default void updateInputs() {}
}
