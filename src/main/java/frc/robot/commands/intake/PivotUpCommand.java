package frc.robot.commands.intake;

import static org.wpilib.units.Units.Volts;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import org.wpilib.command2.Command;

public class PivotUpCommand extends Command {
    private final Intake intake;

    public PivotUpCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPivotVoltage(Volts.of(IntakeConstants.PIVOT_UP_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
    }
}
