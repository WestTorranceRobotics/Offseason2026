package frc.robot.commands.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;

public class OutakeCommand extends Command {
    private final Intake intake;

    public OutakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakeVoltage(Volts.of(IntakeConstants.OUTTAKE_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
