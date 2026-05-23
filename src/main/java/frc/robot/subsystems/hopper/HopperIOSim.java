package frc.robot.subsystems.hopper;

import static frc.robot.constants.HopperConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

public class HopperIOSim implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);
    private final SparkMaxSim hopperMotorSim;

    private final FlywheelSim flywheelSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(2), 0.00062156662, 1), DCMotor.getNEO(2));

    public HopperIOSim() {
        hopperMotorSim = new SparkMaxSim(hopperMotor, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs() {
        updateSim();
    }

    private void updateSim() {
        flywheelSim.setInput(hopperMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        flywheelSim.update(0.02);

        // Update motor
        hopperMotorSim.iterate(flywheelSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(flywheelSim.getCurrentDrawAmps()));
    }

    @Override
    public double getRollerRPM() {
        return flywheelSim.getAngularVelocityRPM();
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        hopperMotor.setVoltage(voltage);
    }
}
