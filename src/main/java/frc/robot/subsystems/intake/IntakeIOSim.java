package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.FlywheelSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Voltage;

public class IntakeIOSim implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(0, INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(0, PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxSim intakeMotorSim;
    private final SparkMaxSim pivotMotorSim;

    // TODO: Find MOI (moment of inertia)
    private final FlywheelSim rollerSim = new FlywheelSim(
            Models.flywheelFromPhysicalConstants(DCMotor.getNEO(1), 0.00062156662, 1), DCMotor.getNEO(1));

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            Models.singleJointedArmFromPhysicalConstants(DCMotor.getNEO(1), 11.8438079981694, 75),
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
        inputs.intakeRPM = Units.radiansPerSecondToRotationsPerMinute(rollerSim.getAngularVelocity());
        inputs.intakeCurrent = rollerSim.getCurrentDraw();

        inputs.pivotRPM = Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocity());
        double pivotAngle = pivotSim.getAngle();
        inputs.pivotPosition = pivotAngle <= PIVOT_ENCODER_POSITION_DEADBAND
                ? "DOWN"
                : (pivotAngle >= (Math.PI / 2 - PIVOT_ENCODER_POSITION_DEADBAND) ? "UP" : "IN BETWEEN");
    }

    private void updateSim() {
        rollerSim.setInput(intakeMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        rollerSim.update(0.02);

        pivotSim.setInput(pivotMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        pivotSim.update(0.02);

        // Update motors
        intakeMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(rollerSim.getAngularVelocity()),
                RoboRioSim.getVInVoltage(),
                0.02);
        pivotMotorSim.iterate(
                Units.radiansPerSecondToRotationsPerMinute(pivotSim.getVelocity()), RoboRioSim.getVInVoltage(), 0.02);

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                rollerSim.getCurrentDraw() + pivotSim.getCurrentDraw()));
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
