package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.HopperConstants.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    public Command runHopperCommand() {
        return this.runEnd(this::runHopper, this::stopHopper);
    }

    public void runHopper() {
        io.setRollerVoltage(Volts.of(HOPPER_VOLTAGE));
    }

    public void stopHopper() {
        io.setRollerVoltage(Volts.of(0));
    }
}
