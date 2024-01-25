package frc.robot.subsystems.shooter;

public class ShooterConstants {
    
  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private SubsystemConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Shooter";

  public static final int RIGHT_SHOOTER_MOTOR_ID = 0;
  public static final int LEFT_SHOOTER_MOTOR_ID = 0;
  public static final int ANGLE_MOTOR_ID = 0;
  public static final int DRUM_MOTOR_ID = 0;
  public static final int CANCODER_ID = 0;

  public static final double SHOOT_KP = 0.0;
  public static final double SHOOT_KI = 0.0;
  public static final double SHOOT_KD = 0.0;
  public static final double SHOOT_PID_PEAK_OUTPUT = 1.0;
  public static final double ROTAION_KP = 0.0;
  public static final double ROTAION_KI = 0.0;
  public static final double ROTAION_KD = 0.0;
  public static final double ROTAION_PID_PEAK_OUTPUT = 1.0;
  public static final double POSITION_FEEDFORWARD = 0;

  public static final double CONTINUOUS_CURRENT_LIMIT = 40;
  public static final double PEAK_CURRENT_LIMIT = 50;
  public static final double PEAK_CURRENT_DURATION = 0.5;
    
}
