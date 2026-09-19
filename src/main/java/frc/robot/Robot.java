// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.firecontrol.FuelPhysicsSim;
import frc.robot.constants.GlobalConstants;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt3D;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;

public class Robot extends LoggedRobot {
    private final RobotContainer robotContainer;
    private Command autonomousCommand;

    public static SimulatedArena3D arena;
    public static FuelPhysicsSim fuelSim;

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // Instantiate all of useful robot code
        Logger.recordMetadata("ProjectName", "Offseason2026");
        Logger.recordMetadata("TeamNumber", "5124");

        switch (GlobalConstants.ROBOT_MODE) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter()); // Logs to USB stick (FAT32 format) connected to the RoboRIO
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                // Logger.addDataReceiver(new WPILOGWriter(".")); // Log to current directory
                Logger.addDataReceiver(new NT4Publisher());
                arena = new Arena2026Rebuilt3D();
                fuelSim = new FuelPhysicsSim("Sim/Fuel");
                break;
            case REPLAY:
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        robotContainer = new RobotContainer();
    }

    /**
     * This function is called every 20 ms, no matter the mode. Use this for items
     * like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard
     * integrated updating.
     */
    public void robotPeriodic() {
        // Do not remove, ticks commands
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    public void disabledInit() {
        robotContainer.clearModuleStates();
    }

    public void disabledPeriodic() {}

    /**
     * Runs at beginning of autonomous.
     * <p>
     * Schedules autonomous command
     */
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    public void autonomousPeriodic() {}

    /** Called at beginning of teleop */
    public void teleopInit() {
        // Makes sure the autonomous command stops running when teleop starts.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    public void teleopPeriodic() {}

    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    public void simulationInit() {
        fuelSim.enable();
        fuelSim.placeFieldBalls();
    }

    /** This function is called periodically whilst in simulation. */
    public void simulationPeriodic() {
        arena.simulationPeriodic();
        fuelSim.tick();
    }
}
