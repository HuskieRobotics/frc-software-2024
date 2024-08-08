package frc.robot.subsystems.note_targeting;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class NoteTargeting extends SubsystemBase {

  private final TunableNumber kP = new TunableNumber("NoteTargeting/kP", 0.025);
  private final TunableNumber kI = new TunableNumber("NoteTargeting/kI", 0.0);
  private final TunableNumber kD = new TunableNumber("NoteTargeting/kD", 0.0);
  private final NoteTargetingIO io;
  private boolean targeting;
  public static final String SUBSYSTEM_NAME = "NoteTargeting";
  private PIDController pid = new PIDController(kP.get(), kI.get(), kD.get());
  private NoteTargetingIOInputsAutoLogged inputs = new NoteTargetingIOInputsAutoLogged();

  public NoteTargeting(NoteTargetingIO io) {
    this.io = io;
    this.targeting = false;
    pid.reset();
    pid.setSetpoint(0.0);
    pid.setTolerance(0.5);
  }

  private void handleLEDs() {
    // we see nothing,  we see it but aren't going after it, we are now going after it

    if (targeting) {
      LEDs.getInstance().requestState(LEDs.States.PURSUING_NOTE);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(SUBSYSTEM_NAME, inputs);

    handleLEDs();

    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged()) {
      pid.setP(kP.get());
      pid.setI(kI.get());
      pid.setD(kD.get());
    }
  }

  public void setTargetingEnabled(boolean enable) {
    if (enable) {
      targeting = true;
    } else {
      targeting = false;
    }
  }

  public double getAdjustment() {
    if (inputs.hasTarget) {
      double adjustment = pid.calculate(inputs.x);
      Logger.recordOutput(SUBSYSTEM_NAME + "/adjustment", adjustment);
      return adjustment;
    }
    return 0.0;
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }
}
