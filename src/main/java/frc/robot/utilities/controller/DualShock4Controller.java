package frc.robot.utilities.controller;

import org.wpilib.command2.button.CommandNiDsPS4Controller;
import org.wpilib.command2.button.Trigger;

public class DualShock4Controller implements Controller {
    private final CommandNiDsPS4Controller ps4Controller;

    public DualShock4Controller(int port) {
        ps4Controller = new CommandNiDsPS4Controller(port);
    }

    @Override
    public double getLeftX() {
        return ps4Controller.getLeftX();
    }

    @Override
    public double getLeftY() {
        return ps4Controller.getLeftY();
    }

    @Override
    public double getRightX() {
        return ps4Controller.getRightX();
    }

    @Override
    public double getRightY() {
        return ps4Controller.getRightY();
    }

    public double getRightAnalogTrigger() {
        return ps4Controller.getR2Axis();
    }

    public Trigger aOrCross() {
        return ps4Controller.cross();
    }

    public Trigger bOrCircle() {
        return ps4Controller.circle();
    }

    public Trigger yOrTriangle() {
        return ps4Controller.triangle();
    }

    public Trigger xOrSquare() {
        return ps4Controller.square();
    }

    public Trigger dPadLeft() {
        return ps4Controller.povLeft();
    }

    public Trigger dPadUp() {
        return ps4Controller.povUp();
    }

    public Trigger dPadRight() {
        return ps4Controller.povRight();
    }

    public Trigger dPadDown() {
        return ps4Controller.povDown();
    }

    @Override
    public Trigger zero() {
        return ps4Controller.R1();
    }
}
