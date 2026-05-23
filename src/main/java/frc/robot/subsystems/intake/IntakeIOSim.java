package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class IntakeIOSim implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxSim intakeMotorSim;
    private final SparkMaxSim pivotMotorSim;

    // TODO: Find MOI (moment of inertia)
    private final FlywheelSim rollerSim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(DCMotor.getNEO(1), 11.8438079981694, 75),
            DCMotor.getNEO(1),
            1.0 / 125,
            6,
            0,
            Math.PI / 2,
            true,
            Math.PI / 2);

    public IntakeIOSim() {
        intakeMotorSim = new SparkMaxSim(intakeMotor, DCMotor.getNEO(1));
        pivotMotorSim = new SparkMaxSim(pivotMotor, DCMotor.getNEO(1));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        updateSim();
        inputs.intakeRPM = rollerSim.getAngularVelocityRPM();
        inputs.pivotRPM = Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocityRadPerSec());
        inputs.pivotPosition = pivotSim.getAngleRads() <= PIVOT_ENCODER_POSITION_DEADBAND
                ? "DOWN"
                : (pivotSim.getAngleRads() >= (Math.PI / 2 - PIVOT_ENCODER_POSITION_DEADBAND) ? "UP" : "IN BETWEEN");
    }

    private void updateSim() {
        rollerSim.setInput(intakeMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        rollerSim.update(0.02);

        pivotSim.setInput(pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        pivotSim.update(0.02);

        // Update motors
        intakeMotorSim.iterate(rollerSim.getAngularVelocityRPM(), RoboRioSim.getVInVoltage(), 0.02);
        pivotMotorSim.iterate(pivotSim.getVelocityRadPerSec(), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                rollerSim.getCurrentDrawAmps() + pivotSim.getCurrentDrawAmps()));
    }

    @Override
    public void setIntakeVoltage(Voltage voltage) {
        intakeMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotMotor.setVoltage(voltage);
    }
}
