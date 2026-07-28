package frc.robot.commands.intake;

import static org.wpilib.units.Units.Volts;

import frc.robot.constants.IntakeConstants;
import frc.robot.subsystems.intake.Intake;
import org.wpilib.command2.Command;

public class IntakeCommand extends Command {
    private final Intake intake;

    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setIntakeVoltage(Volts.of(IntakeConstants.INTAKE_VOLTAGE));
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
