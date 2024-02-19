package frc.robot.subsystems.noteTargeting;

import org.littletonrobotics.junction.AutoLog;

public interface NoteTargetingIO {
  @AutoLog
  public static class NoteTargetingIOInputs {
    public double x = 0.0;
    public double y = 0.0;
    public double a = 0.0;
    public boolean hasTarget = false;
  }

  public void updateInputs(NoteTargetingIOInputs inputs);
}
