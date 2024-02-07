package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {

  @AutoLog
  public static class IntakeIOInputs {
    boolean isRightRollerIRBlocked = false;
    boolean isLeftRollerIRBlocked = false;
    boolean isDrumIRBlocked = false;
    boolean isKickerIRBlocked = false;

    double rightRollerStatorCurrentAmps = 0;
    double leftRollerStatorCurrentAmps = 0;
    double drumStatorCurrentAmps = 0;
    double kickerStatorCurrentAmps = 0;

    double rightRollerSupplyCurrentAmps = 0;
    double leftRollerSupplyCurrentAmps = 0;
    double drumSupplyCurrentAmps = 0;
    double kickerSupplyCurrentAmps = 0;

    double rightRollerVelocityRotationsPerSecond = 0;
    double leftRollerVelocityRotationsPerSecond = 0;
    double drumVelocityRotationsPerSecond = 0;
    double kickerVelocityRotationsPerSecond = 0;

    double rightRollerReferenceVelocityRPS = 0;
    double leftRollerReferenceVelocityRPS = 0;
    double drumReferenceVelocityRPS = 0;
    double kickerReferenceVelocityRPS = 0;

    double leftRollerAppliedVolts = 0;
    double leftRollerClosedLoopError = 0;
    double leftRollerRotorVelocity = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setLeftRollerVelocity(double velocity) {}

  public default void setRightRollerVelocity(double velocity) {}

  public default void setDrumVelocity(double velocity) {}

  public default void setKickerVelocity(double velocity) {}
}
