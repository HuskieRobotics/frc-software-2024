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
    double angleMotorVelocityRPS = 0.0;
    double angleMotorReferenceVelocityRPS = 0.0;
    double angleEncoderAngle = 0.0;

    // Drum Motor Inputs
    double drumMotorStatorCurrentAmps = 0.0;
    double drumMotorSupplyCurrentAmps = 0.0;
    double drumMotorVelocityRPS = 0.0;
    double drumMotorReferenceVelocityRPS = 0.0;

    // Dunker Motor Inputs
    double dunkerMotorStatorCurrentAmps = 0.0;
    double dunkerMotorSupplyCurrentAmps = 0.0;
    double dunkerMotorVelocityRPS = 0.0;
    double dunkerMotorReferenceVelocityRPS = 0.0;

    // Shooter Sensor
    boolean isShooterSensorBlocked = true;    
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setDrumMotorVelocity(double rps) {}

  public default void setDunkerMotorVelocity(double rps) {}

  public default void setShooterMotorLeftVelocity(double rps) {}

  public default void setShooterMotorRightVelocity(double rps) {}

  public default void setAngle(double angle) {}

  public default void getSensor() {}

  public default void getEncoderAngle() {}

  public default void getVelocityRPS() {}

  public default void getStatorCurrent() {}
  
  public default void getSupplyCurrent() {}
}
