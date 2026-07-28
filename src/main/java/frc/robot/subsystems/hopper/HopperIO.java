package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Voltage;

public interface HopperIO {
    @AutoLog
    public class HopperIOInputs {
        public double hopperRPM = 0;
    }

    public void setRollerVoltage(Voltage voltage);

    public default void updateInputs(HopperIOInputs inputs) {}
}
