package frc.robot.subsystems.shooter;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

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
