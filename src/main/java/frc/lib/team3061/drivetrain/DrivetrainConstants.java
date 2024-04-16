package frc.lib.team3061.drivetrain;

public class DrivetrainConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private DrivetrainConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Drivetrain";

  public static final double ANGLE_TO_SPEAKER_TOLERANCE = Math.PI / 36.0;
  
  public static final double VELOCITY_TOLERANCE = 0.2;
  public static final double ANGLE_TOLERANCE = 1;

  public static final double ROTATION_FUTURE_PROJECTION_LEAD_TIME_SECONDS = 0.3;
  public static final double SHOT_DELAY_SECONDS = 0.1;
  public static final double Y_VELOCITY_THRESHOLD_FOR_ROTATION_FUTURE_PROJECTION = 1.5;
}
