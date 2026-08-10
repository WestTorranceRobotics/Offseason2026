package frc.robot.utilities.controller;

import org.wpilib.command2.button.CommandGenericHID;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.POVDirection;

public class LogitechController extends CommandGenericHID implements Controller {
    private GenericHID controller;

    public LogitechController(int port) {
        super(port);
        controller = super.getHID();
    }

    @Override
    public double getLeftX() {
        return controller.getRawAxis(0);
    }

    @Override
    public double getLeftY() {
        return -controller.getRawAxis(1);
    }

    @Override
    public double getRightX() {
        return controller.getRawAxis(4);
    }

    @Override
    public double getRightY() {
        return -controller.getRawAxis(5);
    }

    @Override
    public double getRightAnalogTrigger() {
        return controller.getRawAxis(3);
    }

    @Override
    public double getLeftAnalogTrigger() {
        return controller.getRawAxis(2);
    }

    @Override
    public Trigger aOrCross() {
        return new Trigger(() -> controller.getRawButton(1));
    }

    @Override
    public Trigger bOrCircle() {
        return new Trigger(() -> controller.getRawButton(2));
    }

    @Override
    public Trigger yOrTriangle() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    @Override
    public Trigger xOrSquare() {
        return new Trigger(() -> controller.getRawButton(3));
    }

    @Override
    public Trigger dPadLeft() {
        return new Trigger(() -> controller.getPOV() == POVDirection.LEFT);
    }

    @Override
    public Trigger dPadUp() {
        return new Trigger(() -> controller.getPOV() == POVDirection.UP);
    }

    @Override
    public Trigger dPadRight() {
        return new Trigger(() -> controller.getPOV() == POVDirection.RIGHT);
    }

    @Override
    public Trigger dPadDown() {
        return new Trigger(() -> controller.getPOV() == POVDirection.DOWN);
    }

    @Override
    public Trigger leftBumper() {
        return new Trigger(() -> controller.getRawButton(5));
    }

    @Override
    public Trigger zero() {
        return new Trigger(() -> controller.getRawButton(6));
    }
}
