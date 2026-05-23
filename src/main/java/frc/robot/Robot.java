// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena3D;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt3D;

@Logged
public class Robot extends TimedRobot {
    private final RobotContainer m_robotContainer;
    private Command autonomousCommand;

    public static SimulatedArena3D arena = new Arena2026Rebuilt3D();

    /**
     * This function is run when the robot is first started up and should be used
     * for any
     * initialization code.
     */
    public Robot() {
        // Instantiate all of useful robot code
        DataLogManager.start();
        Epilogue.bind(this);
        m_robotContainer = new RobotContainer();
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
    @Override
    public void robotPeriodic() {
        // Do not remove, ticks commands
        CommandScheduler.getInstance().run();
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {
        m_robotContainer.clearModuleStates();
    }

    @Override
    public void disabledPeriodic() {}

    /**
     * Runs at beginning of autonomous.
     * <p>
     * Schedules autonomous command
     */
    @Override
    public void autonomousInit() {
        autonomousCommand = m_robotContainer.getAutonomousCommand();

        // Ensures that operator cannot move robot during autonomous
        // WARN do not remove, will break path planner
        m_robotContainer.unbindJoystick();
        m_robotContainer.bindAutoCommand();

        if (autonomousCommand != null) CommandScheduler.getInstance().schedule(autonomousCommand);
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** Called at beginning of teleop */
    @Override
    public void teleopInit() {
        // Make sure that our movement command is registered
        m_robotContainer.unbindJoystick();
        m_robotContainer.bindJoystickCommand();

        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        arena.simulationPeriodic();
    }
}
