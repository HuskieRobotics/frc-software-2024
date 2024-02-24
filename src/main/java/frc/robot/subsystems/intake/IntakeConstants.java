package frc.robot.subsystems.intake;

public class IntakeConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private IntakeConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final int INTAKE_ROLLER_MOTOR_ID = 18;
  public static final int INTAKE_KICKER_MOTOR_ID = 21;

  public static final int INTAKE_RIGHT_ROLLER_IR_SENSOR_ID = 0;
  public static final int INTAKE_KICKER_IR_SENSOR_ID = 1;
  public static final int INTAKE_SHOOTER_IR_SENSOR_ID = 2;

  // add all current limiting

  public static final double INTAKE_ROLLER_MOTORS_KP = 10.0;
  public static final double INTAKE_ROLLER_MOTORS_KI = 0.0;
  public static final double INTAKE_ROLLER_MOTORS_KD = 0.0;
  public static final double INTAKE_ROLLER_MOTORS_KS = 6.5;

  public static final double INTAKE_DRUM_MOTOR_KP = 1.0;
  public static final double INTAKE_DRUM_MOTOR_KI = 0.0;
  public static final double INTAKE_DRUM_MOTOR_KD = 0.0;
  public static final double INTAKE_DRUM_MOTOR_KS = 4.0;

  public static final double INTAKE_KICKER_MOTOR_KP = 1.0;
  public static final double INTAKE_KICKER_MOTOR_KI = 0.0;
  public static final double INTAKE_KICKER_MOTOR_KD = 0.0;
  public static final double INTAKE_KICKER_MOTOR_KS = 4.0;

  // velocities

  public static final double INTAKE_VELOCITY_ROLLERS_RPS = 40.0;
  public static final double REPEL_VELOCITY_ROLLERS_RPS = -40.0;
  public static final double KICKER_VELOCITY_RPS = 6.0;

  public static final double ROLLER_VELOCITY_TOLERANCE = 1.0;
  public static final double KICKER_VELOCITY_TOLERANCE = 0.5;

  // current limits

  public static final double ROLLERS_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double ROLLERS_PEAK_CURRENT_LIMIT = 20.0;
  public static final double ROLLERS_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean ROLLERS_MOTOR_INVERTED = false;

  public static final double KICKER_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double KICKER_PEAK_CURRENT_LIMIT = 20.0;
  public static final double KICKER_PEAK_CURRENT_DURATION = 0.1;

  public static final boolean RIGHT_ROLLER_MOTOR_INVERTED = true;
  public static final boolean LEFT_ROLLER_MOTOR_INVERTED = true;
  public static final boolean KICKER_MOTOR_INVERTED = false;

  public static final double ROLLERS_SENSOR_TO_MECHANISM_RATIO = 5.0 / 4.0;

  public static final double IN_BETWEEN_TIMEOUT_SECONDS = 1.0;
}
