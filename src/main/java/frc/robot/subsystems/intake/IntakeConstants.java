package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int INTAKE_RIGHT_ROLLER_MOTOR_ID = 18;
  public static final int INTAKE_LEFT_ROLLER_MOTOR_ID = 19;
  public static final int INTAKE_DRUM_MOTOR_ID = 20;
  public static final int INTAKE_KICKER_MOTOR_ID = 21;

  public static final int INTAKE_RIGHT_ROLLER_IR_SENSOR_ID = 0;
  public static final int INTAKE_LEFT_ROLLER_IR_SENSOR_ID = 1;
  public static final int INTAKE_DRUM_IR_SENSOR_ID = 2;
  public static final int INTAKE_KICKER_IR_SENSOR_ID = 3;

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

  public static final double INTAKE_VELOCITY_ROLLERS_RPS = 70.0;
  public static final double REPEL_VELOCITY_ROLLERS_RPS = -70.0;
  public static final double DRUM_VELOCITY_RPS = 8.0;
  public static final double KICKER_VELOCITY_RPS = 6.0;

  // current limits

  public static final double ROLLERS_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double ROLLERS_PEAK_CURRENT_LIMIT = 20.0;
  public static final double ROLLERS_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean ROLLERS_MOTOR_INVERTED = false;

  public static final double DRUM_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double DRUM_PEAK_CURRENT_LIMIT = 20.0;
  public static final double DRUM_PEAK_CURRENT_DURATION = 0.1;

  public static final double KICKER_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double KICKER_PEAK_CURRENT_LIMIT = 20.0;
  public static final double KICKER_PEAK_CURRENT_DURATION = 0.1;

  public static final boolean RIGHT_ROLLER_MOTOR_INVERTED = true;
  public static final boolean LEFT_ROLLER_MOTOR_INVERTED = true;
  public static final boolean DRUM_MOTOR_INVERTED = false;
  public static final boolean KICKER_MOTOR_INVERTED = false;

  public static final double ROLLERS_SENSOR_TO_MECHANISM_RATIO = 5.0 / 4.0;
}
