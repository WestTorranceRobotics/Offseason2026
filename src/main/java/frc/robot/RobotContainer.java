// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.OutakeCommand;
import frc.robot.commands.intake.PivotDownCommand;
import frc.robot.commands.intake.PivotUpCommand;
import frc.robot.commands.shooter.AlignAndShootCommand;
import frc.robot.commands.shooter.OverrideShootCommand;
import frc.robot.commands.swerve.AlignCommand;
import frc.robot.commands.swerve.DefaultJoystickCommand;
import frc.robot.constants.GlobalConstants.OperatorConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOReal;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConfigurator;
import frc.robot.subsystems.swerve.SwerveIOReal;
import frc.robot.subsystems.swerve.SwerveIOSim;
import frc.robot.subsystems.swerve.gyro.GyroIOReal;
import frc.robot.subsystems.swerve.gyro.GyroIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOReal;
import frc.robot.subsystems.vision.VisionIOSim;
import frc.robot.utilities.controller.Controller;
import frc.robot.utilities.controller.DualShock4Controller;
import frc.robot.utilities.controller.LogitechController;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation3D;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * s, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerveDrive;
    private final Shooter shooter;
    private final Intake intake;
    private final Hopper hopper;

    private final Controller controller;
    private final Controller overrideController;

    public static SwerveDriveSimulation3D swerveDriveSimulation;

    public static Vision vision;

    private final LoggedNetworkNumber shooterRPMTuning = new LoggedNetworkNumber("/Tuning/Shooter RPM", 3000.0);
    private LoggedDashboardChooser<Command> autoChooser;

    /**
     * Registers all important robot code, e.g. swerve, path planner, controls
     */
    public RobotContainer() {
        if (Robot.isReal()) {
            swerveDrive = new Swerve(new SwerveIOReal(), new GyroIOReal(), SwerveConfigurator.createRealModules());

            shooter = new Shooter(new ShooterIOReal());
            intake = new Intake(new IntakeIOReal());
            hopper = new Hopper(new HopperIOReal());
            vision = new Vision(new VisionIOReal(), swerveDrive::addVisionMeasurement);
        } else {
            SwerveIOSim swerveDriveIOSim = new SwerveIOSim();
            swerveDriveSimulation = swerveDriveIOSim.getSimulation();

            swerveDrive = new Swerve(
                    swerveDriveIOSim,
                    new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                    SwerveConfigurator.createSimModules(swerveDriveSimulation));

            shooter = new Shooter(new ShooterIOSim());
            intake = new Intake(new IntakeIOSim());
            hopper = new Hopper(new HopperIOSim());
            vision = new Vision(new VisionIOSim(), swerveDrive::addVisionMeasurement);
        }

        controller = new DualShock4Controller(OperatorConstants.DRIVER_CONTROLLER_PORT);
        overrideController = new LogitechController(OperatorConstants.OVERRIDE_CONTROLLER_PORT);

        registerNamedCommands();

        autoChooser = new LoggedDashboardChooser<>(
                "Auto Routine",
                AutoBuilder.buildAutoChooserWithOptionsModifier(
                        (stream) -> stream.filter(auto -> !auto.getName().startsWith("test"))));

        configureBindings();
    }

    private void registerNamedCommands() {
        NamedCommands.registerCommand(
                "AlignAndShoot",
                new AlignAndShootCommand(() -> 0, () -> 0, swerveDrive, shooter, intake, hopper, vision));
        NamedCommands.registerCommand("Intake", new IntakeCommand(intake));
        NamedCommands.registerCommand("Outake", new OutakeCommand(intake));
        NamedCommands.registerCommand("PivotDown", new PivotDownCommand(intake).withTimeout(0.75));
        NamedCommands.registerCommand("PivotUp", new PivotUpCommand(intake).withTimeout(0.75));
    }

    private void configureBindings() {
        // Sets the controller as the default movement command for swerve.
        swerveDrive.setDefaultCommand(new DefaultJoystickCommand(
                controller::getLeftX, controller::getLeftY, controller::getRightX, swerveDrive));

        // reset the heading of the swerve
        controller.zero().onTrue(Commands.runOnce(swerveDrive::zeroHeading));

        // shooter button mapping
        controller
                .aOrCross()
                .whileTrue(new AlignAndShootCommand(
                        controller::getLeftX, controller::getLeftY, swerveDrive, shooter, intake, hopper, vision));

        // align button mapping
        controller
                .bOrCircle()
                .whileTrue(
                        new AlignCommand(controller::getLeftX, controller::getLeftY, vision::getYawOfHub, swerveDrive));

        controller.xOrSquare().toggleOnTrue(new IntakeCommand(intake));
        controller.yOrTriangle().toggleOnTrue(new OutakeCommand(intake));

        controller.dPadUp().whileTrue(new PivotUpCommand(intake));
        controller.dPadDown().whileTrue(new PivotDownCommand(intake));

        overrideController
                .aOrCross()
                .whileTrue(new OverrideShootCommand(shooter, intake, hopper, ShooterConstants.MINIMUM_SHOOTER_RPM));

        overrideController.bOrCircle().whileTrue(new OverrideShootCommand(shooter, intake, hopper, 3200.0));

        overrideController.xOrSquare().whileTrue(new OverrideShootCommand(shooter, intake, hopper, 3700.0));

        overrideController.yOrTriangle().whileTrue(new OverrideShootCommand(shooter, intake, hopper, 4200.0));
    }

    /**
     * Gets path planner auto to be run during autonomous.
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }

    /**
     * Stops the robot's movement.
     */
    public void clearModuleStates() {
        swerveDrive.drive(new ChassisSpeeds(), true);
    }
}
