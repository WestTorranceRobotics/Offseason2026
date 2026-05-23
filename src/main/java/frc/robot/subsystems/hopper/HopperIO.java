package frc.robot.subsystems.hopper;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {
    @AutoLog
    public class HopperIOInputs {
        public double hopperRPM = 0;
    }

    public void setRollerVoltage(Voltage voltage);

    public default void updateInputs(HopperIOInputs inputs) {}
}
