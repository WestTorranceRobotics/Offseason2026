package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

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
