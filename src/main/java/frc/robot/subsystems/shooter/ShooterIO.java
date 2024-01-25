package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    double power = 0.0;
    double velocity = 0.0;
    double appliedCurrent = 0.0;
    double angle = 0.0;
    boolean isSensorBlocked = true;
    double supplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setAngle(double angle) {}

  public default void setAppliedCurrent(double current) {}

  public default void setMotorPower(double power) {}

  public default void getSensor() {}

  public default void getEncoderAngle() {}

  public default void getVelocity() {}
}
