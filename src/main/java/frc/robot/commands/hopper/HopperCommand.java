package frc.robot.commands.hopper;

import static org.wpilib.units.Units.Volts;

import frc.robot.constants.HopperConstants;
import frc.robot.subsystems.hopper.Hopper;
import org.wpilib.command2.Command;

public class HopperCommand extends Command {
    private final Hopper hopper;

    public HopperCommand(Hopper hopper) {
        this.hopper = hopper;
        addRequirements(hopper);
    }

    @Override
    public void execute() {
        hopper.setHopperVoltage(Volts.of(HopperConstants.HOPPER_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        hopper.stopHopper();
    }
}
