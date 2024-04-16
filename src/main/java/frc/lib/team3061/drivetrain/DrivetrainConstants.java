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

  public static final double ROTATION_FUTURE_PROJECTION_SECONDS = 0.4;
}
