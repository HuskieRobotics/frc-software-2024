package frc.robot.subsystems.climber;

public class ClimberConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String CLIMBER = "Climber";

  public static final int LEFT_MOTOR_CAN_ID = 16;
  public static final int RIGHT_MOTOR_CAN_ID = 17;

  // FIXME: update gear ratio
  public static final double GEAR_RATIO = 25.0;
  public static final double DRUM_CIRCUMFERENCE = 0.0254;
  public static final boolean MOTOR_INVERTED = false;

  public static final double KP = 0.0;
  public static final double KI = 0;
  public static final double KD = 0;

  public static final double KS = 1.0;
  public static final double KV = 0.0;
  public static final double KA = 0.0;
  public static final double KV_EXPO = 0.0;
  public static final double KA_EXPO = 0.0;
  public static final double KG = 0.0; // FIXME: Update all K values

  public static final double CONTINUOUS_CURRENT_LIMIT = 50;
  public static final double PEAK_CURRENT_LIMIT = 60;
  public static final double PEAK_CURRENT_DURATION = 0.5;
  public static final double STATOR_CURRENT_LIMIT = ; // FIXME: Update stator current limit. Software feature sheet is not updated

  public static final int TIMEOUT_MS = 30;
}
