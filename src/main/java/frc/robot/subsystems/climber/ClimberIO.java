package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Generic subsystem hardware interface. */
public interface ClimberIO {
  /** Contains all of the input data received from hardware. */
  @AutoLog
  public static class ClimberIOInputs {
    double leftVelocityRPS = 0.0;
    double leftPositionRotationsMeters = 0.0;
    double leftStatorCurrentAmps = 0.0;
    double leftSupplyCurrentAmps = 0.0;
    double leftSetpoint = 0.0;

    double rightVelocityRPS = 0.0;
    double rightPositionRotationsMeters = 0.0;
    double rightStatorCurrentAmps = 0.0;
    double rightSupplyCurrentAmps = 0.0;
    double rightSetpoint = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public default void setLeftMotorPosition(double position) {}

  public default void setRightMotorPosition(double position) {}

  public default void setPositionZero() {}

  public default void setLeftMotorCurrent(double current) {}

  public default void setRightMotorCurrent(double current) {}

  public default void enableLongArms()  {}

  public default void disableLongArms()  {}

  public default boolean getLongArms()  {
    return false;
  }
}
