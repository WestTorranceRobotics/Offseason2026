package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);

    public IntakeIOReal() {
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(INTAKE_MOTOR_CURRENT_LIMIT);
        intakeConfig.idleMode(IdleMode.kCoast);
        intakeConfig.inverted(true);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.smartCurrentLimit(PIVOT_MOTOR_CURRENT_LIMIT);
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotConfig.encoder.positionConversionFactor(2 * Math.PI); // Converts from rotations to radians
        pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // We're going to manually start the robot with the intake pivot on top of the robot.
        // I'm setting the convention that this starting position is 90 degrees (pi/2 radians).
        // Our target angle (when the intake is down, level with the floor) should be zero degrees,
        pivotMotor.getEncoder().setPosition(Math.PI / 2);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeRPM = intakeMotor.getEncoder().getVelocity();
        inputs.pivotRPM = pivotMotor.getEncoder().getVelocity();
        inputs.pivotPosition = pivotMotor.getEncoder().getPosition() <= PIVOT_ENCODER_POSITION_DEADBAND
                ? "DOWN"
                : (pivotMotor.getEncoder().getPosition() >= (Math.PI / 2 - PIVOT_ENCODER_POSITION_DEADBAND)
                        ? "UP"
                        : "IN BETWEEN");
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
