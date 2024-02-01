package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    double leftVelocityRPM = 0.0;
    double leftPosition = 0.0;
    double leftStatorCurrentAmps = 0.0;
    double leftAngleDegrees = 0.0;
    double leftSetpoint = 0.0;
    double leftPower = 0.0;

    double rightVelocityRPM = 0.0;
    double rightPosition = 0.0;
    double rightStatorCurrentAmps = 0.0;
    double rightAngleDegrees = 0.0;
    double rightSetpoint = 0.0;
    double rightPower = 0.0;

    double leftReferenceVelocity = 0.0;
    double leftReferenceSetpoint = 0.0;
    double leftReferencPower = 0.0;

    double rightReferenceVelocity = 0.0;
    double rightReferenceSetpoint = 0.0;
    double rightReferencePower = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public default void setLeftMotorPower(double power) {}

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public default void setLeftMotorCurrent(double current) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setLeftMotorPosition(double position, double arbitraryFeedForward) {}

  // Methods for right motor below
  public default void setRightMotorPower(double power) {}

  public default void setRightMotorCurrent(double current) {}

  public default void setRightMotorPosition(double position, double arbitraryFeedForward) {}

  public default void getAngle() {}

}
