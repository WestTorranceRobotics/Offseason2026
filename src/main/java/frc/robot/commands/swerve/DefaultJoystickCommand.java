package frc.robot.commands.swerve;

import static frc.robot.utilities.controller.InputProcessing.*;

import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;
import org.wpilib.command2.Command;
import org.wpilib.math.kinematics.ChassisVelocities;

public class DefaultJoystickCommand extends Command {
    private final DoubleSupplier ly;
    private final DoubleSupplier lx;
    private final DoubleSupplier rx;
    private final Swerve drive;

    /**
     * @param lx          Translation on the x-axis supplier
     * @param ly          Translation on the y-axis supplier
     * @param rx          Desired rotation velocity supplier
     * @param swerveDrive Swerve drive train instance
     */
    public DefaultJoystickCommand(DoubleSupplier lx, DoubleSupplier ly, DoubleSupplier rx, Swerve swerveDrive) {
        this.lx = lx;
        this.ly = ly;
        this.rx = rx;

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
                -curve(applyDeadband(rx.getAsDouble())) * SwerveDriveConstants.MAX_ANGULAR_SPEED);

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
