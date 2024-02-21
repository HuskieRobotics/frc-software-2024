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

  public static final double GEAR_RATIO = 25.0;
  public static final double DRUM_CIRCUMFERENCE_METERS = 0.0254;
  public static final boolean LEFT_MOTOR_INVERTED = false;
  public static final boolean RIGHT_MOTOR_INVERTED = true;

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
  public static final double STATOR_CURRENT_LIMIT =
      0; // FIXME: Update stator current limit. Software feature sheet is not updated

  public static final int TIMEOUT_MS = 30;
  public static final double LEFT_EXTEND_POSITION = 0;
  public static final double RIGHT_EXTEND_POSITION = 0;
  public static final double LEFT_CONTINUE_POSITION = 0;
  public static final double RIGHT_CONTINUE_POSITION = 0;
}