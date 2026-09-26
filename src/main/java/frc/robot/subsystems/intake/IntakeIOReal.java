package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static org.wpilib.units.Units.Volts;

import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.units.measure.Voltage;

public class IntakeIOReal implements IntakeIO {
    private final SparkMax intakeMotor = new SparkMax(0, INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax pivotMotor = new SparkMax(0, PIVOT_MOTOR_ID, MotorType.kBrushless);

    private final SlewRateLimiter intakeLimiter = new SlewRateLimiter(INTAKE_SLEW_RATE_LIMITER);

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
        inputs.intakeRPM = intakeMotor.getEncoder().getVelocity().get();
        inputs.intakeCurrent = intakeMotor.getOutputCurrent().get();

        inputs.pivotRPM = pivotMotor.getEncoder().getVelocity().get();
        double pivotPosition = pivotMotor.getEncoder().getPosition().get();
        inputs.pivotPosition = pivotPosition <= PIVOT_ENCODER_POSITION_DEADBAND
                ? "DOWN"
                : (pivotPosition >= (Math.PI / 2 - PIVOT_ENCODER_POSITION_DEADBAND) ? "UP" : "IN BETWEEN");
    }

    @Override
    public void setIntakeVoltage(Voltage voltage) {
        intakeMotor.setVoltage(intakeLimiter.calculate(voltage.in(Volts)));
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotMotor.setVoltage(voltage);
    }
}
