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
    intakeState = getIntakeState();
  }

  public IntakeStates getIntakeState() {
    if (inputs.isDrumIRBlocked) {
      return IntakeStates.DRUMFILLED;
    } else if (inputs.isRightRollerIRBlocked && inputs.isLeftRollerIRBlocked) {
      return IntakeStates.INTAKINGBOTHEMERGENCY;
    } else if (inputs.isRightRollerIRBlocked) {
      return IntakeStates.INTAKINGLEFT;
    } else if (inputs.isLeftRollerIRBlocked) {
      return IntakeStates.INTAKINGRIGHT;
    } else {
      return IntakeStates.EMPTY;
    }
  }

  public IntakeStates getCurrentIntakeState() {
    return intakeState;
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
