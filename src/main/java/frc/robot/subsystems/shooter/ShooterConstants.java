package frc.robot.subsystems.shooter;

public class ShooterConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private ShooterConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Shooter";
  public static final boolean DEFLECTOR_ENABLED = false;

  public static final boolean USE_MATHEMATICAL_MODEL = false;

  public static final int TOP_SHOOTER_MOTOR_ID = 27;
  public static final int BOTTOM_SHOOTER_MOTOR_ID = 28;
  public static final int ANGLE_MOTOR_ID = 29;
  public static final int ANGLE_ENCODER_ID = 9;
  public static final int DEFLECTOR_MOTOR_ID = 30; // not decided yet?

  public static final int COAST_BUTTON_ID = 5;

  public static final int SET_POINT_COUNT = 5;

  // Shooter Velocity PID Constants
  public static final double TOP_SHOOT_KP = 5.0;
  public static final double TOP_SHOOT_KI = 0.0;
  public static final double TOP_SHOOT_KD = 0.0;
  public static final double TOP_SHOOT_KS = 6.0;
  public static final double TOP_SHOOT_PID_PEAK_OUTPUT = 1.0;
  public static final double BOTTOM_SHOOT_KP = 5.0;
  public static final double BOTTOM_SHOOT_KI = 0.0;
  public static final double BOTTOM_SHOOT_KD = 0.0;
  public static final double BOTTOM_SHOOT_KS = 7.0;
  public static final double BOTTOM_SHOOT_PID_PEAK_OUTPUT = 1.0;

  // Shooter Rotation PID Constants
  public static final double ROTATION_KP = 50;
  public static final double ROTATION_KI = 0.0;
  public static final double ROTATION_KD = 10;
  public static final double ROTATION_KS = 0.16206;
  public static final double ROTATION_KG = 0.22544;
  public static final double ROTATION_KV = 20.835;
  public static final double ROTATION_KA = 0.16507;
  public static final double ROTATION_EXPO_KV = 80.0;
  public static final double ROTATION_EXPO_KA = 24.0;
  public static final double ROTATION_PID_PEAK_OUTPUT = 1.0;

  // Shoot Motors
  public static final double SHOOT_MOTOR_TOP_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT = 60;
  public static final double SHOOT_MOTOR_TOP_PEAK_CURRENT_DURATION = 0.1;

  public static final double SHOOT_MOTOR_BOTTOM_CONTINUOUS_CURRENT_LIMIT = 30;
  public static final double SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT = 60;
  public static final double SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_DURATION = 0.1;

  public static final double SHOOT_MOTORS_GEAR_RATIO = 0.5;
  public static final double SHOOTER_IDLE_VELOCITY = 10.0;
  public static final boolean SHOOT_TOP_INVERTED = false;
  public static final boolean SHOOT_BOTTOM_INVERTED = false;
  public static final double VELOCITY_TOLERANCE = 5.0; // FIXME: tune with real robot
  public static final double SUBWOOFER_RANGE_VELOCITY_TOP = 60;
  public static final double SUBWOOFER_RANGE_VELOCITY_BOTTOM = 60;
  public static final double NEAR_RANGE_VELOCITY_TOP = 80;
  public static final double NEAR_RANGE_VELOCITY_BOTTOM = 80;
  public static final double MID_RANGE_VELOCITY_TOP = 100;
  public static final double MID_RANGE_VELOCITY_BOTTOM = 100;
  public static final double FAR_RANGE_VELOCITY_TOP = 120;
  public static final double FAR_RANGE_VELOCITY_BOTTOM = 120;
  public static final double SUBWOOFER_TO_NEAR_VELOCITY_DISTANCE_METERS = 2.0;
  public static final double NEAR_TO_MID_VELOCITY_DISTANCE_METERS = 2.895;
  public static final double MID_TO_FAST_VELOCITY_DISTANCE_METERS = 4.071;

  // Deflector Motor
  public static final double DEFLECTOR_DEPLOY_VOLTAGE = 1;
  public static final double DEFLECTOR_RETRACT_VOLTAGE = -1;
  public static final double DEFLECTOR_MOTOR_CONTINUOUS_CURRENT_LIMIT = 10;
  public static final double DEFLECTOR_MOTOR_PEAK_CURRENT_LIMIT = 15;
  public static final double DEFLECTOR_MOTOR_PEAK_CURRENT_DURATION = 0.1;
  public static final boolean DEFLECTOR_MOTOR_INVERTED = false;
  public static final double DEFLECTOR_RETRACTION_DELAY_SECS = 0.2;

  // Angle Motor
  public static final double ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT = 15;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_LIMIT = 20;
  public static final double ANGLE_MOTOR_PEAK_CURRENT_DURATION = 0.1;
  public static final double ANGLE_MOTOR_STATOR_CURRENT_LIMIT_TIME = 0.5;
  public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0;
  public static final double ANGLE_MOTOR_GEAR_RATIO = 45.0;
  public static final boolean ANGLE_MOTOR_INVERTED = true;
  public static final double MAGNET_OFFSET = -0.70727;
  public static final double SENSOR_TO_MECHANISM_RATIO = 4.0;
  // FIXME: tune on practice field and reduce this value
  public static final double ANGLE_TOLERANCE_DEGREES = 1.5;
  public static final double SHOOTER_STORAGE_ANGLE = 14.7;
  public static final double MAX_INTAKE_ANGLE = 45.0; // FIXME: tune
  public static final double ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE = 1.0;
  public static final double UPPER_ANGLE_LIMIT = 135.0;

  // 0.074951 rotations = 10.4 degrees

  // Shooter Presets
  public static final double SUBWOOFER_VELOCITY_TOP = 60;
  public static final double SUBWOOFER_VELOCITY_BOTTOM = 60;
  public static final double SUBWOOFER_ANGLE = 64;
  public static final double PODIUM_VELOCITY_TOP = 120;
  public static final double PODIUM_VELOCITY_BOTTOM = 120;
  public static final double PODIUM_ANGLE = 40;
  public static final double AMP_VELOCITY_TOP = 35; // 30;
  public static final double AMP_VELOCITY_BOTTOM = 22; // 17;
  public static final double AMP_ANGLE = 52; // 56;
  public static final double PASS_VELOCITY_TOP = 60.0;
  public static final double PASS_VELOCITY_BOTTOM = 60.0;
  public static final double PASS_ANGLE = 48.0;
  public static final double SOURCE_SIDE_AUTO_1_VELOCITY = 100.0;
  public static final double SOURCE_SIDE_AUTO_1_ANGLE = 30.5;
  public static final double SOURCE_SIDE_AUTO_2_VELOCITY = 120.0;
  public static final double SOURCE_SIDE_AUTO_2_ANGLE = 28.6;
  public static final double SOURCE_SIDE_AUTO_3_4_VELOCITY = 100.0;
  public static final double SOURCE_SIDE_AUTO_3_4_ANGLE = 31.6;
  public static final double SPEAKER_AUTO_OUTBOUND_VELOCITY = 100.0;
  public static final double SPEAKER_AUTO_OUTBOUND_ANGLE = 35.0; // FIXME: tune
  public static final double SPEAKER_AUTO_INBOUND_VELOCITY = 100.0;
  public static final double SPEAKER_AUTO_INBOUND_ANGLE = 35.0; // FIXME: tune
  public static final double SPEAKER_AUTO_3_VELOCITY = 100.0;
  public static final double SPEAKER_AUTO_3_ANGLE = 45.0;
  public static final double SPEAKER_AUTO_4_VELOCITY = 100.0;
  public static final double SPEAKER_AUTO_4_ANGLE = 43.7;
  public static final double AMP_SIDE_AUTO_1_VELOCITY = 100.0;
  public static final double AMP_SIDE_AUTO_1_ANGLE = 30.5;
  public static final double AMP_SIDE_AUTO_2_VELOCITY = 120.0;
  public static final double AMP_SIDE_AUTO_2_ANGLE = 28.8;
  public static final double AMP_SIDE_AUTO_3_VELOCITY = 80.0;
  public static final double AMP_SIDE_AUTO_3_ANGLE = 37.5;
  public static final double AMP_SIDE_AUTO_4_VELOCITY = 80.0;
  public static final double AMP_SIDE_AUTO_4_ANGLE = 39.5;
  public static final double AMP_SIDE_AUTO_5_VELOCITY = 80.0;
  public static final double AMP_SIDE_AUTO_5_ANGLE = 41;
  public static final double AMP_SIDE_AUTO_6_VELOCITY = 80.0;
  public static final double AMP_SIDE_AUTO_6_ANGLE = 39.5;
  public static final double AMP_FAR_SIDE_AUTO_1_VELOCITY = 120.0;
  public static final double AMP_FAR_SIDE_AUTO_1_ANGLE = 33.0; // FIXME: tune

  // auto shot
  public static final double SHOOTER_AUTO_SHOT_ANGLE_DEG = 29.5;
  public static final double SHOOTER_AUTO_SHOT_VELOCITY_RPS = 120.0;
  public static final double SHOOTER_AUTO_SHOT_DISTANCE_METERS = 5.06;
  public static final double SHOOTER_AUTO_SHOT_TOLERANCE_METERS = 0.1;
  public static final double SHOOTER_AUTO_SHOT_TIME_DELAY_SECS = 0.2;
}
