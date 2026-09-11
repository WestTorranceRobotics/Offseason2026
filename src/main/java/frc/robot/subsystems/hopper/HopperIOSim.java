package frc.robot.subsystems.hopper;

import static frc.robot.constants.HopperConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Voltage;

public class HopperIOSim implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(0, HOPPER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxSim hopperMotorSim;

    // TODO: Find MOI (moment of inertia)
    private final FlywheelSim flywheelSim = new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    public HopperIOSim() {
        hopperMotorSim = new SparkMaxSim(hopperMotor, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        updateSim();
        inputs.hopperRPM = hopperMotor.getEncoder().getVelocity().get();
    }

    private void updateSim() {
        flywheelSim.setInput(hopperMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motor
        hopperMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(flywheelSim.getAngularVelocity()),
                RoboRioSim.getVInVoltage(),
                0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDraw()));
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        hopperMotor.setVoltage(voltage);
    }
}
