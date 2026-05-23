package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class ShooterIOSim implements ShooterIO {
    private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless); // dummy
    private final SparkMax flywheelMotor = new SparkMax(LAUNCHER_MOTOR_1_ID, MotorType.kBrushless);
    private final SparkMax flywheelMotorInverted = new SparkMax(LAUNCHER_MOTOR_2_ID, MotorType.kBrushless);

    private final SparkMaxSim feederMotorSim;
    private final SparkMaxSim launcherMotorLeaderSim;
    private final SparkMaxSim launcherMotorFollowerSim;

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(3), 0.00062156662, 1), DCMotor.getNEO(3));

    private final FlywheelSim feederSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    public ShooterIOSim() {
        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));
        launcherMotorLeaderSim = new SparkMaxSim(flywheelMotor, DCMotor.getNEO(1));
        launcherMotorFollowerSim = new SparkMaxSim(flywheelMotorInverted, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs() {
        updateSim();
    }

    private void updateSim() {
        flywheelSim.setInput(launcherMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        feederSim.update(0.02);

        // Update motors
        feederMotorSim.iterate(feederSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorLeaderSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorFollowerSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                flywheelSim.getCurrentDrawAmps() + feederSim.getCurrentDrawAmps()));
    }

    @Override
    public double getFlywheelRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public double getFeederRPM() {
        return feederSim.getAngularVelocityRPM();
    }

    @Override
    public void setFlywheelVoltage(Voltage voltage) {
        flywheelMotor.setVoltage(voltage);
        flywheelMotorInverted.setVoltage(voltage);
    }

    @Override
    public void setFeederVoltage(Voltage voltage) {
        feederMotor.setVoltage(voltage);
    }
}
