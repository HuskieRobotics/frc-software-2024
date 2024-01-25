package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputs inputs = new IntakeIOInputs();

  public enum IntakeStates {
    INTAKINGRIGHT,
    INTAKINGLEFT,
    DRUMFILLED,
    EMPTY,
    INTAKINGBOTHEMERGENCY
  }

  private IntakeStates intakeState;

  public Intake(IntakeIO io) {
    this.io = io;
    intakeState = IntakeStates.EMPTY;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public boolean getRightRollerIRSensor() {
    return inputs.isRightRollerIRBlocked;
  }

  public boolean getLeftRollerIRSensor() {
    return inputs.isLeftRollerIRBlocked;
  }

  public boolean getDrumIRSensor() {
    return inputs.isDrumIRBlocked;
  }

  public void setRightRollerPower(double percentage) {
    io.setRightRollerPower(percentage);
  }

  public void setLeftRollerPower(double percentage) {
    io.setLeftRollerPower(percentage);
  }

  public void setDrumPower(double percentage) {
    io.setDrumPower(percentage);
  }

  public void setRightRollerCurrent(double current) {
    io.setRightRollerCurrent(current);
  }

  public void setLeftRollerCurrent(double current) {
    io.setLeftRollerCurrent(current);
  }

  public void setDrumCurrent(double current) {
    io.setDrumCurrent(current);
  }
}
