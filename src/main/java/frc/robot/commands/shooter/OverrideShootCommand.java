package frc.robot.commands.shooter;

import static frc.robot.utilities.CustomUnits.RotationsPerMinute;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class OverrideShootCommand extends Command {
    private final Shooter shooter;
    private final Hopper hopper;
    private final double targetRPM;

    public OverrideShootCommand(Shooter shooter, Hopper hopper, double targetRPM) {
        this.shooter = shooter;
        this.hopper = hopper;
        this.targetRPM = targetRPM;

        addRequirements(shooter, hopper);
    }

    @Override
    public void execute() {
        shooter.setFlywheelSpeed(RotationsPerMinute.of(targetRPM));
        shooter.runFeeder();
        hopper.runHopper();
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopShooter();
        hopper.stopHopper();
    }
}
