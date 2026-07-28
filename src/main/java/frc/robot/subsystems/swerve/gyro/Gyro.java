package frc.robot.subsystems.swerve.gyro;

import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;

public class Gyro {
    private final GyroIO io;
    private final GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

    public Gyro(GyroIO io) {
        this.io = io;
    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Gyro", inputs);
    }

    public Rotation2d getRotation() {
        return inputs.rotation2D;
    }
}
