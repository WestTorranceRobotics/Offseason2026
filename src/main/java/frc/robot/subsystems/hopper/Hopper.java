package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.CustomUnits;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private double actualRPM = 0;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs();
        actualRPM = io.getRollerRPM();
        SmartDashboard.putNumber("Hopper RPM", actualRPM);
    }

    public AngularVelocity getRollerSpeed() {
        return CustomUnits.RotationsPerMinute.of(actualRPM);
    }

    public void setHopperVoltage(Voltage voltage) {
        io.setRollerVoltage(voltage);
    }

    public void stopHopper() {
        io.setRollerVoltage(Volts.of(0));
    }
}
