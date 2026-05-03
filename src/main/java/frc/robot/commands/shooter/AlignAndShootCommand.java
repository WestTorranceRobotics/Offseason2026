package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class AlignAndShootCommand extends ParallelCommandGroup {
    public AlignAndShootCommand(
            DoubleSupplier lx, DoubleSupplier ly, Swerve swerveDrive, Shooter shooter, Vision vision, Hopper hopper) {
        addCommands(
                new AlignCommand(lx, ly, () -> -swerveDrive.getAngleToHub().getDegrees(), swerveDrive),
                new ShootCommand(shooter, swerveDrive, vision, hopper));
    }
}
