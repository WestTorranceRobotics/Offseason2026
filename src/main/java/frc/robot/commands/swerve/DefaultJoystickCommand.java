package frc.robot.commands.swerve;

import static frc.robot.constants.GlobalConstants.OperatorConstants.DEADBAND_THRESHOLD;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.SwerveDriveConstants;
import frc.robot.subsystems.swerve.Swerve;
import java.util.function.DoubleSupplier;

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
     * @param rawInput The input that the deadband should be applied to
     * @return The given input where if the magnitude is below the deadband threshold it is equal to
     * zero
     */
    private double applyDeadband(double rawInput) {
        return Math.abs(rawInput) < DEADBAND_THRESHOLD ? 0 : rawInput;
    }

    private double curve(double input) {
        return Math.signum(input) * Math.pow(Math.abs(input), 2);
    }

    /**
     * Send controller data to the drive train.
     */
    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                -curve(applyDeadband(ly.getAsDouble())) * SwerveDriveConstants.MAX_TRANSLATION_SPEED,
                -curve(applyDeadband(lx.getAsDouble())) * SwerveDriveConstants.MAX_TRANSLATION_SPEED,
                -curve(applyDeadband(rx.getAsDouble())) * SwerveDriveConstants.MAX_ANGULAR_SPEED);

        drive.drive(chassisSpeeds, true);

        drive.tickPid();
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
