package frc.robot.subsystems.noteTargeting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class NoteTargeting extends SubsystemBase {

  private final TunableNumber kP = new TunableNumber("NoteTargeting/kP", 0.1);
  private final TunableNumber kI = new TunableNumber("NoteTargeting/kI", 0.0);
  private final TunableNumber kD = new TunableNumber("NoteTargeting/kD", 0.0);
  private final NoteTargetingIO leftio;
  private final NoteTargetingIO rightio;
  public final String SUBSYSTEM_NAME = "NoteTargeting";
  private NoteTargetingIOInputsAutoLogged primaryTarget = null;
  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());
  private NoteTargetingIOInputsAutoLogged leftInputs = new NoteTargetingIOInputsAutoLogged();
  private NoteTargetingIOInputsAutoLogged rightInputs = new NoteTargetingIOInputsAutoLogged();

  public NoteTargeting(NoteTargetingIO leftio, NoteTargetingIO rightio) {
    this.leftio = leftio;
    this.rightio = rightio;
    pid.reset();
    pid.setSetpoint(0.0);
    pid.setTolerance(0.5);
  }

  // in case where both target are same size we will use the target we already have locked on
  public void assignPrimaryTarget() {
    if (leftInputs.hasTarget || rightInputs.hasTarget) {
      if (leftInputs.a > rightInputs.a) {
        primaryTarget = leftInputs;
      } else if (rightInputs.a > leftInputs.a) {
        primaryTarget = rightInputs;
      }
    }
  }

  public void handleLEDs() {}

  @Override
  public void periodic() {
    leftio.updateInputs(leftInputs);
    rightio.updateInputs(rightInputs);
    assignPrimaryTarget();
    handleLEDs();
    pid.setP(kP.get());
    pid.setI(kI.get());
    pid.setD(kD.get());
    Logger.processInputs(SUBSYSTEM_NAME + "/left", leftInputs);
    Logger.processInputs(SUBSYSTEM_NAME + "/right", rightInputs);
  }

  public double getRotateVelocity() {
    if (leftInputs.hasTarget || rightInputs.hasTarget) {
      return pid.calculate(primaryTarget.x);
    }
    return 0.0;
  }
}
