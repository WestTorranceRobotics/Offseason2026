package frc.robot.subsystems.shooter;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;

@Logged
public interface ShooterIO {
    public double getFlywheelRPM();

    public double getFeederRPM();

    public void setFlywheelVoltage(Voltage voltage);

    public void setFeederVoltage(Voltage voltage);

    default void updateInputs() {}
    ;
}
