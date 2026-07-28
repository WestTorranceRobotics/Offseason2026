package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Voltage;

public interface ShooterIO {
    @AutoLog
    public class ShooterIOInputs {
        public double flywheelRPM = 0;
        public double feederRPM = 0;
    }

    public void setFlywheelVoltage(Voltage voltage);

    public void setFeederVoltage(Voltage voltage);

    public default void updateInputs(ShooterIOInputs inputs) {}
    ;
}
