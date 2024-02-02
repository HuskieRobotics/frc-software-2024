package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int RIGHT_SHOOTER_MOTOR_ID = 0;
  public static final int LEFT_SHOOTER_MOTOR_ID = 0;
  public static final int ANGLE_MOTOR_ID = 0;
  public static final int DRUM_MOTOR_ID = 0;
  public static final int DUNKER_MOTOR_ID = 0;
  public static final int ANGLE_ENCODER_ID = 0;
  public static final int SHOOTER_SENSOR_ID = 0;

  // Shooter Velocity PID Constants
  public static final double SHOOT_KP = 0.0;
  public static final double SHOOT_KI = 0.0;
  public static final double SHOOT_KD = 0.0;
  public static final double SHOOT_PID_PEAK_OUTPUT = 1.0;

  // Shooter Rotation PID Constants
  public static final double ROTATION_KP = 0.0;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 0.0;
  public static final double ROTATION_PID_PEAK_OUTPUT = 1.0;

  // Shoot Motors
  public static final double SHOOT_MOTORS_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double SHOOT_MOTORS_PEAK_CURRENT_LIMIT = 50;
  public static final double SHOOT_MOTORS_PEAK_CURRENT_DURATION = 0.5;
  public static final boolean SHOOT_LEFT_INVERTED = false;
  public static final boolean SHOOT_RIGHT_INVERTED = false; 

  // Right Shooter Motor
  // public static final double SHOOT_RIGHT_CONTINUOUS_CURRENT_LIMIT = 40;
  // public static final double SHOOT_RIGHT_PEAK_CURRENT_LIMIT = 50;
  // public static final double SHOOT_RIGHT_PEAK_CURRENT_DURATION = 0.5;

  // Angle Motor
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 50;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.5;
  public static final boolean ANGLE_MOTOR_INVERTED = false;

  // Drum Motor
  public static final double DRUM_MOTOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double DRUM_MOTOR_PEAK_CURRENT_LIMIT = 50;
  public static final double DRUM_MOTOR_PEAK_CURRENT_DURATION = 0.5;
  public static final boolean DRUM_MOTOR_INVERTED = false;

  // Dunker Motor
  public static final double DUNKER_MOTOR_CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double DUNKER_MOTOR_PEAK_CURRENT_LIMIT = 50;
  public static final double DUNKER_MOTOR_PEAK_CURRENT_DURATION = 0.5;
  public static final boolean DUNKER_MOTOR_INVERTED = false;

  // Feed Forward Constants (More Need to be added)
  public static final double POSITION_FEEDFORWARD = 0.0;


}
