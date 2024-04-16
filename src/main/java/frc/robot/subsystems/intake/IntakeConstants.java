package frc.robot.subsystems.intake;

public class IntakeConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private IntakeConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean TESTING = false;

  public static final int INTAKE_ROLLER_MOTOR_ID = 18;
  public static final int INTAKE_KICKER_MOTOR_ID = 21;

  public static final int INTAKE_ROLLER_IR_SENSOR_ID = 1; // want: 3; backup: 9
  public static final int INTAKE_KICKER_IR_SENSOR_ID = 2; // want: 2 backup: 6
  public static final int INTAKE_SHOOTER_IR_SENSOR_ID = 3; // want: 0; backup: 3

  public static final double INTAKE_ROLLER_MOTORS_KP = 10.0;
  public static final double INTAKE_ROLLER_MOTORS_KI = 0.0;
  public static final double INTAKE_ROLLER_MOTORS_KD = 0.0;
  public static final double INTAKE_ROLLER_MOTORS_KS = 6.5;

  public static final double INTAKE_KICKER_MOTOR_KP = 10.0;
  public static final double INTAKE_KICKER_MOTOR_KI = 0.0;
  public static final double INTAKE_KICKER_MOTOR_KD = 0.0;
  public static final double INTAKE_KICKER_MOTOR_KS = 13.0;

  // velocities

  public static final double INTAKE_VELOCITY_ROLLERS_RPS = 40.0;
  public static final double REPEL_VELOCITY_ROLLERS_RPS = -40.0;

  // ideally, we would match the linear speed of the kicker wheel to that of the intake rollers,
  // however, there is a potential to move the note too far such that it contacts the shooter wheels
  // and is shot prematurely. To mitigate this risk, we will run the kicker at a slower linear
  // velocity than the intake.
  public static final double KICKER_INTAKING_VELOCITY_RPS = INTAKE_VELOCITY_ROLLERS_RPS / 5;
  public static final double KICKER_SHOOTING_VELOCITY_VOLTAGE = 12.0;

  public static final double ROLLER_VELOCITY_TOLERANCE = 1.0;
  public static final double KICKER_VELOCITY_TOLERANCE = 0.5;

  // current limits

  public static final double ROLLERS_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 15.0;
  public static final double ROLLERS_PEAK_SUPPLY_CURRENT_LIMIT = 20.0;
  public static final double ROLLERS_PEAK_SUPPLY_CURRENT_DURATION = 0.1;

  public static final double ROLLERS_CONTINUOUS_STATOR_CURRENT_LIMIT = 40.0;

  public static final double KICKER_CONTINUOUS_SUPPLY_CURRENT_LIMIT = 30.0;
  public static final double KICKER_PEAK_SUPPLY_CURRENT_LIMIT = 40.0;
  public static final double KICKER_PEAK_SUPPLY_CURRENT_DURATION = 1.0;

  public static final boolean ROLLER_MOTOR_INVERTED = false;
  public static final boolean KICKER_MOTOR_INVERTED = false;

  public static final double ROLLERS_SENSOR_TO_MECHANISM_RATIO = 15.0 / 12.0;
  public static final double KICKER_SENSOR_TO_MECHANISM_RATIO = 5.0;

  public static final double IN_BETWEEN_TIMEOUT_SECONDS = 1.0;
}
