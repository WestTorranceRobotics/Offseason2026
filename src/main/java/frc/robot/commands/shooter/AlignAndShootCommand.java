package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.hopper.HopperCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class AlignAndShootCommand extends ParallelCommandGroup {
    public AlignAndShootCommand(
            DoubleSupplier lx,
            DoubleSupplier ly,
            Swerve swerveDrive,
            Shooter shooter,
            Intake intake,
            Hopper hopper,
            Vision vision) {
        addCommands(
                new AlignCommand(lx, ly, () -> -swerveDrive.getShootingAngle().getDegrees(), swerveDrive),
                new ShootCommand(shooter, swerveDrive, vision),
                new IntakeCommand(intake),
                new HopperCommand(hopper));
    }
}
