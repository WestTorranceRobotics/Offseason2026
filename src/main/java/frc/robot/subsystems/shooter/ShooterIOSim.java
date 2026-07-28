package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Voltage;

public class ShooterIOSim implements ShooterIO {
    private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, 0, MotorType.kBrushless);
    private final SparkMax flywheelMotor = new SparkMax(LAUNCHER_MOTOR_1_ID, 0, MotorType.kBrushless);
    private final SparkMax flywheelMotorInverted = new SparkMax(LAUNCHER_MOTOR_2_ID, 0, MotorType.kBrushless);

    private final SparkMaxSim feederMotorSim;
    private final SparkMaxSim launcherMotorLeaderSim;
    private final SparkMaxSim launcherMotorFollowerSim;

    // TODO: Find MOI (moment of inertia)
    private final FlywheelSim flywheelSim = new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getNEO(2), 0.00062156662, 1), DCMotor.getNEO(2));

    private final FlywheelSim feederSim = new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    public ShooterIOSim() {
        feederMotorSim = new SparkMaxSim(feederMotor, DCMotor.getNEO(1));
        launcherMotorLeaderSim = new SparkMaxSim(flywheelMotor, DCMotor.getNEO(1));
        launcherMotorFollowerSim = new SparkMaxSim(flywheelMotorInverted, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        updateSim();
        inputs.flywheelRPM = flywheelMotor.getEncoder().getVelocity().get();
        inputs.feederRPM = feederMotor.getEncoder().getVelocity().get();
    }

    private void updateSim() {
        flywheelSim.setInput(launcherMotorLeaderSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        feederSim.setInput(feederMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        feederSim.update(0.02);

        // Update motors
        feederMotorSim.iterate(feederSim.getAngularVelocity(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorLeaderSim.iterate(flywheelSim.getAngularVelocity(), RoboRioSim.getVInVoltage(), 0.02);
        launcherMotorFollowerSim.iterate(flywheelSim.getAngularVelocity(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                flywheelSim.getCurrentDraw() + feederSim.getCurrentDraw()));
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
