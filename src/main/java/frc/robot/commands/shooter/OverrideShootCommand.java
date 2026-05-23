package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ShooterConstants.FEEDER_VOLTAGE;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hopper.HopperCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class OverrideShootCommand extends ParallelCommandGroup {
    public OverrideShootCommand(Shooter shooter, Intake intake, Hopper hopper, double targetRPM) {
        addCommands(
                shooter.runEnd(
                        () -> {
                            shooter.setFlywheelSpeed(RPM.of(targetRPM));
                            shooter.setFeederVoltage(Volts.of(FEEDER_VOLTAGE));
                        },
                        () -> {
                            shooter.stopShooter();
                        }),
                new IntakeCommand(intake),
                new HopperCommand(hopper));
    }
}
