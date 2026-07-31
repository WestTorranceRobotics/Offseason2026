package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.Voltage;

public class Hopper extends SubsystemBase {
    private final HopperIO io;

    private final HopperIOInputsAutoLogged inputs = new HopperIOInputsAutoLogged();

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public void setHopperVoltage(Voltage voltage) {
        io.setRollerVoltage(voltage);
    }

    public void stopHopper() {
        io.setRollerVoltage(Volts.of(0));
    }

    public double getHopperRPM() {
        return inputs.hopperRPM;
    }
}
