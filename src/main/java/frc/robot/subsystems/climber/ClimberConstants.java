package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public class ClimberConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TUNING = false;
  public static final String SUBSYSTEM_NAME = "Climber";

  public static final int CLIMBER_MOTOR_ID = 16;
  public static final boolean CLIMBER_MOTOR_INVERTED = false;
  public static final double CLIMBER_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double CLIMBER_STATOR_CURRENT_LIMIT = 60;
  public static final double CLIMBER_PEAK_CURRENT_LIMIT = 60;
  public static final double CLIMBER_PEAK_CURRENT_DURATION = 2.0;
  public static final double GEAR_RATIO = 45;

  public static final double CIRCUMFERENCE_OF_SPOOL = Units.inchesToMeters(1.0) * Math.PI;

  public static final double EXTENDED_POSITION_ROT = 7.0; // FIXME: tune
  public static final double EXTENDING_VOLTAGE = 3.0; // FIXME: TUNE
  public static final double RETRACTING_VOLTAGE = 3.0; // FIXME: tune

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
