package frc.robot.subsystems.shooter;

import static frc.robot.constants.ShooterConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.wpilib.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {
    private final SparkMax feederMotor = new SparkMax(FEEDER_MOTOR_ID, 0, MotorType.kBrushless);
    private final SparkMax flywheelMotor = new SparkMax(LAUNCHER_MOTOR_1_ID, 0, MotorType.kBrushless);
    private final SparkMax flywheelMotorInverted = new SparkMax(LAUNCHER_MOTOR_2_ID, 0, MotorType.kBrushless);

    public ShooterIOReal() {
        // feeder config
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
        feederConfig.inverted(true);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // launcher motor configs
        SparkMaxConfig flywheelConfig = new SparkMaxConfig();
        flywheelConfig.idleMode(IdleMode.kCoast);
        flywheelConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        flywheelConfig.inverted(false);
        flywheelMotor.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelConfig.inverted(true);
        flywheelMotorInverted.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.flywheelRPM = flywheelMotor.getEncoder().getVelocity().get();
        inputs.feederRPM = feederMotor.getEncoder().getVelocity().get();
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
