package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = true;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int TOP_SHOOTER_MOTOR_ID = 27;
  public static final int BOTTOM_SHOOTER_MOTOR_ID = 28;
  public static final int ANGLE_MOTOR_ID = 29;
  public static final int ANGLE_ENCODER_ID = 9;

  // Shooter Velocity PID Constants
  public static final double SHOOT_KP = 40.0;
  public static final double SHOOT_KI = 0.0;
  public static final double SHOOT_KD = 0.0;
  public static final double SHOOT_KS = 14.0;
  public static final double SHOOT_PID_PEAK_OUTPUT = 1.0;

  // Shooter Rotation PID Constants
  public static final double ROTATION_KP = 40.0;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 1.0;
  public static final double ROTATION_KS = 1.0;
  public static final double ROTATION_KG = 0.08;
  public static final double ROTATION_KV = 1.0;
  public static final double ROTATION_KA = 0.01;
  public static final double ROTATION_EXPO_KV = 24.0;
  public static final double ROTATION_EXPO_KA = 24.0;
  public static final double ROTATION_PID_PEAK_OUTPUT = 1.0;

  // Shoot Motors
  public static final double SHOOT_MOTORS_CONTINUOUS_CURRENT_LIMIT = 20;
  public static final double SHOOT_MOTORS_PEAK_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTORS_PEAK_CURRENT_DURATION = 0.1;
  public static final double SHOOT_MOTORS_GEAR_RATIO = 2;
  public static final double SHOOTER_IDLE_VELOCITY = 0.0;
  public static final boolean SHOOT_TOP_INVERTED = false;
  public static final boolean SHOOT_BOTTOM_INVERTED = false;
  public static final double VELOCITY_TOLERANCE = 0.1;

  // Angle Motor
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 20;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
  public static final double ANGLE_MOTOR_STATOR_CURRENT_LIMIT = 30;
  public static final double ANGLE_MOTOR_STATOR_CURRENT_PEAK = 40;
  public static final double ANGLE_MOTOR_STATOR_CURRENT_LIMIT_TIME = 0.5;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0;
  public static final double ANGLE_MOTOR_GEAR_RATIO = 144.0;
  public static final boolean ANGLE_MOTOR_INVERTED = false;
  public static final double MAGNET_OFFSET = 0.0;
  public static final double SENSOR_TO_MECHANISM_RATIO = 4.0;
  public static final double ANGLE_TOLERANCE = 0.5;

  // Shooter Presets
  public static final double SUBWOOFER_VELOCITY = 10.0;
  public static final double SUBWOOFER_ANGLE = 0.0;
  public static final double PODIUM_VELOCITY = 10.0;
  public static final double PODIUM_ANGLE = 0.0;

}
