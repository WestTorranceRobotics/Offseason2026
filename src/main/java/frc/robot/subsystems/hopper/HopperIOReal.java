package frc.robot.subsystems.hopper;

import static frc.robot.constants.HopperConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public class HopperIOReal implements HopperIO {
    private final SparkMax hopperMotor = new SparkMax(HOPPER_MOTOR_ID, MotorType.kBrushless);

    public HopperIOReal() {
        SparkMaxConfig hopperConfig = new SparkMaxConfig();
        hopperConfig.smartCurrentLimit(HOPPER_MOTOR_CURRENT_LIMIT);
        hopperConfig.idleMode(IdleMode.kCoast);
        hopperConfig.inverted(true);
        hopperMotor.configure(hopperConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public double getRollerRPM() {
        return hopperMotor.getEncoder().getVelocity();
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        hopperMotor.setVoltage(voltage);
    }
}
