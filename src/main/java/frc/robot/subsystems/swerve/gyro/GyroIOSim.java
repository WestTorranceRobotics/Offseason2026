package frc.robot.subsystems.swerve.gyro;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroIOSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true; // Assume gyro is always connected in simulation
        inputs.rotation2D = gyroSimulation.getGyroReading();
        inputs.angularVelocity = gyroSimulation.getMeasuredAngularVelocity();
    }
}
