package frc.robot.commands.swerve;

import static frc.robot.utilities.InputProcessing.*;

import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.kinematics.ChassisVelocities;

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
        ChassisVelocities chassisVelocities = new ChassisVelocities(
                -curve(applyDeadband(ly.getAsDouble())) * SwerveDriveConstants.MAX_TRANSLATION_SPEED,
                -curve(applyDeadband(lx.getAsDouble())) * SwerveDriveConstants.MAX_TRANSLATION_SPEED,
                -alignPIDController.calculate(0, relativeYaw.getAsDouble()) * SwerveDriveConstants.MAX_ANGULAR_SPEED);

        drive.drive(chassisVelocities, true);
    }

    /**
     * Called when the command is stopped.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        drive.drive(new ChassisVelocities(), true);
    }
}
