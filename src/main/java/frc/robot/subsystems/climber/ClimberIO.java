package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    double leftVelocityRPS = 0.0;
    double leftPositionMeters = 0.0;
    double leftStatorCurrentAmps = 0.0;
    double leftSupplyCurrentAmps = 0.0;
    double leftCurrentSetpoint = 0.0;
    double leftReferenceSetpointMeters = 0.0;

    double rightVelocityRPS = 0.0;
    double rightPositionMeters = 0.0;
    double rightStatorCurrentAmps = 0.0;
    double rightSupplyCurrentAmps = 0.0;
    double rightCurrentSetpoint = 0.0;
    double rightReferenceSetpointMeters = 0.0;
    }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  public default void setLeftMotorPosition(double position) {}

  public default void setRightMotorPosition(double position) {}
}
