package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public static class ShooterIOInputs {

    // Top Shooter Motor Inputs
    double shootMotorTopStatorCurrentAmps = 0.0;
    double shootMotorTopSupplyCurrentAmps = 0.0;
    double shootMotorTopVelocityRPS = 0.0;
    double shootMotorTopReferenceVelocityRPS = 0.0;
    double shootMotorTopTemperatureCelsius = 0.0;
    double shootMotorTopVoltage = 0.0;
    double shootMotorTopClosedLoopReferenceSlope = 0.0;

    // Bottom Shooter Motor Inputs
    double shootMotorBottomStatorCurrentAmps = 0.0;
    double shootMotorBottomSupplyCurrentAmps = 0.0;
    double shootMotorBottomVelocityRPS = 0.0;
    double shootMotorBottomReferenceVelocityRPS = 0.0;
    double shootMotorBottomTemperatureCelsius = 0.0;
    double shootMotorBottomVoltage = 0.0;
    double shootMotorBottomClosedLoopReferenceSlope = 0.0;

    // Angle Motor Inputs
    double angleMotorStatorCurrentAmps = 0.0;
    double angleMotorSupplyCurrentAmps = 0.0;
    double angleMotorVoltage = 0.0;
    double angleMotorReferenceAngleDegrees = 0.0;
    double angleEncoderAngleDegrees = 0.0;
    double angleMotorTemperatureCelsius = 0.0;
    double angleMotorClosedLoopReferenceSlope = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterWheelTopVelocity(double rps) {}

  public default void setShooterWheelBottomVelocity(double rps) {}

  public default void setAngle(double angle) {}
}
