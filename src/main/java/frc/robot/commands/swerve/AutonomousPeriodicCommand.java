package frc.robot.commands.swerve;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Swerve;

public class AutonomousPeriodicCommand extends Command {
    private final Swerve swerveDrive;

    public AutonomousPeriodicCommand(Swerve swerveDrive) {
        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.tickPid();
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.drive(new ChassisSpeeds(), true);
        swerveDrive.tickPid();
    }
}
