package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

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

  public static final int SET_POINT_COUNT = 5;

  // Shooter Velocity PID Constants
  public static final double TOP_SHOOT_KP = 40.0;
  public static final double TOP_SHOOT_KI = 0.0;
  public static final double TOP_SHOOT_KD = 0.0;
  public static final double TOP_SHOOT_KS = 14.0;
  public static final double TOP_SHOOT_PID_PEAK_OUTPUT = 1.0;
  public static final double BOTTOM_SHOOT_KP = 40.0;
  public static final double BOTTOM_SHOOT_KI = 0.0;
  public static final double BOTTOM_SHOOT_KD = 0.0;
  public static final double BOTTOM_SHOOT_KS = 14.0;
  public static final double BOTTOM_SHOOT_PID_PEAK_OUTPUT = 1.0;

  // Shooter Rotation PID Constants
  public static final double ROTATION_KP = 40.0;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 1.0;
  public static final double ROTATION_KS = 1.0;
  public static final double ROTATION_KG = 0.08;
  public static final double ROTATION_KV = 1.0;
  public static final double ROTATION_KA = 0.01;
  public static final double ROTATION_EXPO_KV = 80.0;
  public static final double ROTATION_EXPO_KA = 24.0;
  public static final double ROTATION_PID_PEAK_OUTPUT = 1.0;

  // Shoot Motors
  public static final double SHOOT_MOTOR_TOP_CONTINUOUS_CURRENT_LIMIT = 20;
  public static final double SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTOR_TOP_PEAK_CURRENT_DURATION = 0.1;

  public static final double SHOOT_MOTOR_BOTTOM_CONTINUOUS_CURRENT_LIMIT = 20;
  public static final double SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_DURATION = 0.1;

  public static final double SHOOT_MOTORS_GEAR_RATIO = 0.5;
  public static final double SHOOTER_IDLE_VELOCITY = 10.0; // FIXME tune with real robot
  public static final boolean SHOOT_TOP_INVERTED = false;
  public static final boolean SHOOT_BOTTOM_INVERTED = false;
  public static final double VELOCITY_TOLERANCE = 0.1;
  public static final double CLOSE_RANGE_VELOCITY = 20;
  public static final double FAR_RANGE_VELOCITY = 50;
  public static final double VELOCITY_ZONE_SWITCH_DISTANCE = Units.inchesToMeters(106.901) / 2.0;

  // Angle Motor
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 20;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
  public static final double ANGLE_MOTOR_STATOR_CURRENT_LIMIT_TIME = 0.5;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0;
  public static final double ANGLE_MOTOR_GEAR_RATIO = 45.0;
  public static final boolean ANGLE_MOTOR_INVERTED = false;
  public static final double MAGNET_OFFSET = 0.0;
  public static final double SENSOR_TO_MECHANISM_RATIO = 4.0;
  public static final double ANGLE_TOLERANCE_DEGREES = 0.5; // FIXME tune with real robot
  public static final double SHOOTER_STORAGE_ANGLE = 0.0; // FIXME tune with real robot

  // Shooter Presets
  public static final double SUBWOOFER_VELOCITY = 10.0; // FIXME tune with real robot
  public static final double SUBWOOFER_ANGLE = 0.0; // FIXME tune with real robot
  public static final double PODIUM_VELOCITY = 10.0; // FIXME tune with real robot
  public static final double PODIUM_ANGLE = 0.0; // FIXME tune with real robot
  public static final double AMP_VELOCITY = 10.0; // FIXME tune with real robot
  public static final double AMP_ANGLE = 0.0; // FIXME tune with real robot
}
