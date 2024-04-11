package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean isRollerIRBlocked = false;
    boolean isKickerIRBlocked = false;
    boolean isShooterIRBlocked = false;

    double rollerStatorCurrentAmps = 0;
    double kickerStatorCurrentAmps = 0;

    double rollerSupplyCurrentAmps = 0;
    double kickerSupplyCurrentAmps = 0;

    double rollerVelocityRPS = 0;
    double kickerVelocityRPS = 0;

    double rollerReferenceVelocityRPS = 0;
    double kickerReferenceVelocityRPS = 0;

    double rollerTempCelsius = 0;
    double kickerTempCelsius = 0;

    double rollerVoltage = 0;
    double kickerVoltage = 0;

    boolean DIO0 = false;
    boolean DIO1 = false;
    boolean DIO2 = false;
    boolean DIO3 = false;
    boolean DIO4 = false;
    boolean DIO5 = false;
    boolean DIO6 = false;
    boolean DIO7 = false;
    boolean DIO8 = false;
    boolean DIO9 = false;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVelocity(double velocity) {}

  public default void setKickerVelocity(double velocity) {}

  public default void setKickerVoltage(double voltage) {}
}
