package frc.robot.subsystems.intake;

public interface IntakeIO {

  public static class IntakeIOInputs {
    boolean isRightRollerIRBlocked = false;
    boolean isLeftRollerIRBlocked = false;
    boolean isDrumIRBlocked = false;

    double rightRollerAppliedPercentage = 0.0; // dutyCycle
    double leftRollerAppliedPercentage = 0.0; // dutyCycle
    double drumAppliedPercentage = 0.0; // dutyCycle

    double[] rightRollerStatorCurrentAmps = new double[] {};
    double[] leftRollerStatorCurrentAmps = new double[] {};
    double[] drumStatorCurrentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRightRollerPower(double percentage) {}

  public default void setLeftRollerPower(double percentage) {}

  public default void setDrumPower(double percentage) {}

  public default void setRightRollerCurrent(double current) {}

  public default void setLeftRollerCurrent(double current) {}

  public default void setDrumCurrent(double current) {}
}
