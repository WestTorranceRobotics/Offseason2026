package frc.robot.commands.swerve;

import static frc.robot.utilities.controller.InputProcessing.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

public class AlignCommand extends Command {
    private final DoubleSupplier ly;
    private final DoubleSupplier lx;
    private final DoubleSupplier relativeYaw;
    private final Swerve drive;

    private final PIDController alignPIDController =
            new PIDController(SwerveDriveConstants.ALIGN_P, SwerveDriveConstants.ALIGN_I, SwerveDriveConstants.ALIGN_D);

    /**
     * @param lx          Translation on the x-axis supplier
     * @param ly          Translation on the y-axis supplier
     * @param relativeYaw Desired rotation supplier
     * @param swerveDrive Swerve drive train instance
     */
    public AlignCommand(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier relativeYaw, Swerve swerveDrive) {
        this.lx = lx;
        this.ly = ly;
        this.relativeYaw = relativeYaw;

        this.drive = swerveDrive;

        addRequirements(swerveDrive);
    }

    /**
     * Send controller data to the drive train.
     */
    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                -curve(applyDeadband(ly.getAsDouble())) * SwerveDriveConstants.MAX_TRANSLATION_SPEED,
                -curve(applyDeadband(lx.getAsDouble())) * SwerveDriveConstants.MAX_TRANSLATION_SPEED,
                -alignPIDController.calculate(0, relativeYaw.getAsDouble()) * SwerveDriveConstants.MAX_ANGULAR_SPEED);

        drive.drive(chassisSpeeds, true);
    }

    /**
     * Called when the command is stopped.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisSpeeds(), true);
    }
}
