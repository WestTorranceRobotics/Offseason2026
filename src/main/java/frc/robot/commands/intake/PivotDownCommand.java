package frc.robot.commands.intake;

import static org.wpilib.units.Units.Volts;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import org.wpilib.command2.Command;

public class PivotDownCommand extends Command {
    private final Intake intake;

    public PivotDownCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPivotVoltage(Volts.of(IntakeConstants.PIVOT_DOWN_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopPivot();
    }
}
