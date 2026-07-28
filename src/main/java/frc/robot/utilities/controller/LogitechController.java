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

    public double getRightAnalogTrigger() {
        return controller.getRawAxis(3);
    }

    public Trigger aOrCross() {
        return new Trigger(() -> controller.getRawButton(1));
    }

    public Trigger bOrCircle() {
        return new Trigger(() -> controller.getRawButton(2));
    }

    public Trigger yOrTriangle() {
        return new Trigger(() -> controller.getRawButton(4));
    }

    public Trigger xOrSquare() {
        return new Trigger(() -> controller.getRawButton(3));
    }

    public Trigger dPadLeft() {
        return new Trigger(() -> controller.getPOV() == POVDirection.LEFT);
    }

    public Trigger dPadUp() {
        return new Trigger(() -> controller.getPOV() == POVDirection.UP);
    }

    public Trigger dPadRight() {
        return new Trigger(() -> controller.getPOV() == POVDirection.RIGHT);
    }

    public Trigger dPadDown() {
        return new Trigger(() -> controller.getPOV() == POVDirection.DOWN);
    }

    public Trigger R1() {
        return new Trigger(() -> controller.getRawButton(6));
    }

    @Override
    public Trigger zero() {
        return this.R1();
    }
}
