package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {

    public static final boolean DEBUGGING = false;
    public static final boolean TUNING = false;
    public static final String SUBSYSTEM_NAME = "Climber";

    public static final int CLIMBER_MOTOR_ID = 0;
    public static final boolean CLIMBER_MOTOR_INVERTED = false;
    public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 0;
    public static final double CLIMBER_STATOR_CURRENT_LIMIT = 0;
    public static final double CLIMBER_PEAK_CURRENT_LIMIT = 0;
    public static final double CLIMBER_PEAK_CURRENT_DURATION = 0;
    public static final double GEAR_RATIO = 25;

    public static final double CIRCUMFERENCE_OF_SPOOL = Units.inchesToMeters(0)*Math.PI;
    public static final double EXTENDED_POSITION = 0;

    public static final double CLIMB_UP = 1;

    public static final double KP = 0.0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 1.0;
    public static final double KV = 0.0;
    public static final double KA = 0.0;
    public static final double KV_EXPO = 0.0;
    public static final double KA_EXPO = 0.0;
    public static final double KG = 0.0; // FIXME: Update all K values
}
