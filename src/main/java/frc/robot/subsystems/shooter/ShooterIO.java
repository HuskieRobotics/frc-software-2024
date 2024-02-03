package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  
  @AutoLog
  public static class ShooterIOInputs {
    
    // Left Shooter Motor Inputs
    double shootMotorLeftStatorCurrentAmps = 0.0;
    double shootMotorLeftSupplyCurrentAmps = 0.0;
    double shootMotorLeftVelocityRPS = 0.0;
    double shootMotorLeftReferenceVelocityRPS = 0.0;

    // Right Shooter Motor Inputs
    double shootMotorRightStatorCurrentAmps = 0.0;
    double shootMotorRightSupplyCurrentAmps = 0.0;
    double shootMotorRightVelocityRPS = 0.0;
    double shootMotorRightReferenceVelocityRPS = 0.0;

    // Angle Motor Inputs
    double angleMotorStatorCurrentAmps = 0.0;
    double angleMotorSupplyCurrentAmps = 0.0;
    double angleMotorReferenceAngleDegrees  = 0.0;
    double angleEncoderAngleDegrees = 0.0;

    // Kicker Motor Inputs
    double kickerMotorStatorCurrentAmps = 0.0;
    double kickerMotorSupplyCurrentAmps = 0.0;
    double kickerMotorVelocityRPS = 0.0;
    double kickerMotorReferenceVelocityRPS = 0.0;

    // Dunker Motor Inputs
    double dunkerMotorStatorCurrentAmps = 0.0;
    double dunkerMotorSupplyCurrentAmps = 0.0;
    double dunkerMotorVelocityRPS = 0.0;
    double dunkerMotorReferenceVelocityRPS = 0.0;

    // Shooter Sensor
    boolean isKickerSensorBlocked = true;    
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setKickerWheelVelocity(double rps) {}

  public default void setDunkerMotorVelocity(double rps) {}

  public default void setShooterWheelLeftVelocity(double rps) {}

  public default void setShooterWheelRightVelocity(double rps) {}

  public default void setAngle(double angle) {}

  public default void getSensor() {}

}
