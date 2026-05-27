package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.swerve.SwerveConfigurator;

public class ModuleIOReal implements ModuleIO {
    private final SwerveConfigurator.SwerveDriveModuleConstants moduleConstants;

    private final TalonFX driveMotorController;
    private final SparkMax steerMotorController;
    private final CANcoder CANCoder;

    // TODO Document this
    public ModuleIOReal(SwerveConfigurator.SwerveDriveModuleConstants moduleConstants) {
        this.moduleConstants = moduleConstants;

        // Drive motor config
        driveMotorController = new TalonFX(moduleConstants.driveMotorID);

        TalonFXConfigurator driveMotorConfigurator = driveMotorController.getConfigurator();
        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = moduleConstants.DRIVE_P;
        slot0Configs.kI = moduleConstants.DRIVE_I;
        slot0Configs.kD = moduleConstants.DRIVE_D;
        slot0Configs.kV = moduleConstants.DRIVE_V;
        slot0Configs.kS = moduleConstants.DRIVE_S;
        driveMotorConfigurator.apply(slot0Configs);
        driveMotorConfigurator.apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(80)
                .withSupplyCurrentLimit(60)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(40));
        driveMotorConfigurator.apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

        // Azimuth motor config
        steerMotorController = new SparkMax(moduleConstants.azimuthMotorID, SparkMax.MotorType.kBrushless);
        SparkMaxConfig sparkMaxConfig = new SparkMaxConfig();
        // TODO figure out this open loop ramp rate
        sparkMaxConfig
                .smartCurrentLimit(40)
                .idleMode(SparkBaseConfig.IdleMode.kCoast)
                .openLoopRampRate(0.2);
        steerMotorController.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // CANCoder config
        CANCoder = new CANcoder(moduleConstants.CANCoderID);
        CANcoderConfiguration configuration = new CANcoderConfiguration();
        configuration.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        configuration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        configuration.MagnetSensor.MagnetOffset = -moduleConstants.CANCoderOffset;
        CANCoder.getConfigurator().apply(configuration);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.driveVoltage = driveMotorController.getMotorVoltage().getValue().magnitude();
        inputs.steerVoltage = steerMotorController.getAppliedOutput() * RobotController.getBatteryVoltage();

        inputs.driveWheelPositionRotations = driveMotorController
                .getPosition()
                .getValue()
                .times(moduleConstants.driveGearRatio)
                .in(Rotations);
        inputs.driveWheelVelocityRPS = driveMotorController
                .getVelocity()
                .getValue()
                .times(moduleConstants.driveGearRatio)
                .in(RotationsPerSecond);

        inputs.steerAngleRad = new Rotation2d(CANCoder.getAbsolutePosition().getValue()).getRadians();
        inputs.steerVelocityRadPerSec = CANCoder.getVelocity().getValue().in(RadiansPerSecond);
    }

    @Override
    public void setDriveVoltage(Voltage voltage) {
        driveMotorController.setVoltage(voltage.in(Volts));
    }

    @Override
    public void setSteerVoltage(Voltage voltage) {
        steerMotorController.setVoltage(voltage);
    }
}
