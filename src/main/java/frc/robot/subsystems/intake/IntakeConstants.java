package frc.robot.subsystems.intake;

public class IntakeConstants {
  public static final int INTAKE_RIGHT_ROLLER_MOTOR_ID = 0;
  public static final int INTAKE_LEFT_ROLLER_MOTOR_ID = 1;
  public static final int INTAKE_DRUM_MOTOR_ID = 2;

  public static final int INTAKE_RIGHT_ROLLER_IR_SENSOR_ID = 3;
  public static final int INTAKE_LEFT_ROLLER_IR_SENSOR_ID = 4;
  public static final int INTAKE_DRUM_IR_SENSOR_ID = 5;

  // add all current limiting

  public static final double INTAKE_RIGHT_ROLLER_MOTOR_KP = 0.0;
  public static final double INTAKE_RIGHT_ROLLER_MOTOR_KI = 0.0;
  public static final double INTAKE_RIGHT_ROLLER_MOTOR_KD = 0.0;

  public static final double INTAKE_LEFT_ROLLER_MOTOR_KP = 0.0;
  public static final double INTAKE_LEFT_ROLLER_MOTOR_KI = 0.0;
  public static final double INTAKE_LEFT_ROLLER_MOTOR_KD = 0.0;

  public static final double INTAKE_DRUM_MOTOR_KP = 0.0;
  public static final double INTAKE_DRUM_MOTOR_KI = 0.0;
  public static final double INTAKE_DRUM_MOTOR_KD = 0.0;

  //

  public static final double ROLLERS_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double ROLLERS_PEAK_CURRENT_LIMIT = 20.0;
  public static final double ROLLERS_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean ROLLERS_MOTOR_INVERTED = false;

  public static final double DRUM_CONTINUOUS_CURRENT_LIMIT = 15.0;
  public static final double DRUM_PEAK_CURRENT_LIMIT = 20.0;
  public static final double DRUM_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean DRUM_MOTOR_INVERTED = false;
}
