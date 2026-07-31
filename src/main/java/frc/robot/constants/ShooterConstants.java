package frc.robot.constants;

import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
    public static final int FEEDER_MOTOR_ID = 49;
    public static final int LAUNCHER_MOTOR_1_ID = 22;
    public static final int LAUNCHER_MOTOR_2_ID = 3;

    // Current limit values for shooter motors
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 40;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 50;

    public static final double FEEDER_VOLTAGE = 12;

    public static final double LATENCY_COMPENSATION = 0; // TODO tune latency compensation

    public static final double TOLERANCE_TO_RUN_FEEDER = 200;

    // Shooter mount position relative to robot center
    public static final double SHOOTER_X_OFFSET = -0.30;
    public static final double SHOOTER_Y_OFFSET = 0.0;
    public static final double SHOOTER_Z_OFFSET = 0.30;

    // Launch angle in radians (I think it's either 55 or 60)
    public static final double LAUNCH_ANGLE = Math.toRadians(60.0);

    // Rough conversion from flywheel RPM to ball exit velocity
    public static final double RPM_TO_EXIT_VELOCITY = 0.002;

    public static final double YAW_ACCEPTABLE_ERROR = 2;

    /* 2700 is the RPM value for shooting directly next to the Hub. We fall back to it so we can still
    shoot from a known position if vision breaks or is disabled. */
    public static final double MINIMUM_SHOOTER_RPM = 2700.0;

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_TOF_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap SHOOTER_MAP = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap DISTANCE_VS_RPM_MAP = new InterpolatingDoubleTreeMap();

    static {
        // TODO build lookup tables
        DISTANCE_TO_TOF_MAP.put(1.5, 0.42);
        DISTANCE_TO_TOF_MAP.put(2.0, 0.51);
        DISTANCE_TO_TOF_MAP.put(2.5, 0.58);
        DISTANCE_TO_TOF_MAP.put(3.0, 0.65);
        DISTANCE_TO_TOF_MAP.put(3.5, 0.71);
        DISTANCE_TO_TOF_MAP.put(4.0, 0.78);
        DISTANCE_TO_TOF_MAP.put(4.5, 0.84);
        DISTANCE_TO_TOF_MAP.put(5.0, 0.91);

        SHOOTER_MAP.put(3.57142857, 2800.0);
        SHOOTER_MAP.put(3.92156863, 3100.0);
        SHOOTER_MAP.put(4.31034483, 3400.0);
        SHOOTER_MAP.put(4.61538462, 3650.0);
        SHOOTER_MAP.put(4.92957746, 3900.0);
        SHOOTER_MAP.put(5.12820513, 4100.0);
        SHOOTER_MAP.put(5.35714286, 4350.0);
        SHOOTER_MAP.put(5.49450549, 4550.0);

        DISTANCE_VS_RPM_MAP.put(1.0, 2778.0); // INTERPOLATED
        DISTANCE_VS_RPM_MAP.put(2.0, 3200.0);
        DISTANCE_VS_RPM_MAP.put(3.067, 3650.0);
        //        DISTANCE_VS_RPM_MAP.put(4.5, 4116.6667); // INTERPOLATED
        //        DISTANCE_VS_RPM_MAP.put(5.5, 4567.0); // INTERPOLATED
    }
}
