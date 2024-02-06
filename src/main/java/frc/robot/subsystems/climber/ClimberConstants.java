package frc.robot.subsystems.climber;

public class ClimberConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ClimberConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String CLIMBER = "Climber";

  public static final int LEFT_MOTOR_CAN_ID = 15;
  public static final int RIGHT_MOTOR_CAN_ID = 16;

  // FIXME: update gear ratio
  public static final double GEAR_RATIO = 25.0;
  public static final boolean MOTOR_INVERTED = false;

  public static final double LEFT_POSITION_PID_P = 0.0;
  public static final double LEFT_POSITION_PID_I = 0;
  public static final double LEFT_POSITION_PID_D = 0;
  public static final double LEFT_POSITION_PID_KS = 1.0;
  public static final double LEFT_POSITION_PID_KV = 0.0;
  public static final double LEFT_POSITION_PID_KA = 0.0;
  public static final double LEFT_POSITION_PID_KV_EXPO = 0.0;
  public static final double LEFT_POSITION_PID_KA_EXPO = 0.0;

  public static final double RIGHT_POSITION_PID_P = 0.0;
  public static final double RIGHT_POSITION_PID_I = 0;
  public static final double RIGHT_POSITION_PID_D = 0;
  public static final double RIGHT_POSITION_PID_KS = 1.0;
  public static final double RIGHT_POSITION_PID_KV = 0.0;
  public static final double RIGHT_POSITION_PID_KA = 0.0;
  public static final double RIGHT_POSITION_PID_KV_EXPO = 0.0;
  public static final double RIGHT_POSITION_PID_KA_EXPO = 0.0;

  public static final double CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double PEAK_CURRENT_LIMIT = 50;
  public static final double PEAK_CURRENT_DURATION = 0.2;

  public static final int TIMEOUT_MS = 30;
  public static final int SLOT_INDEX = 0;
}
