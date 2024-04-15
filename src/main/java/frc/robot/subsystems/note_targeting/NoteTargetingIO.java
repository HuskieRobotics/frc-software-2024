package frc.robot.subsystems.note_targeting;

import org.littletonrobotics.junction.AutoLog;

public interface NoteTargetingIO {
  @AutoLog
  public static class NoteTargetingIOInputs {
    double x = 0.0;
    double y = 0.0;
    double a = 0.0;
    boolean hasTarget = false;
  }

  public default void updateInputs(NoteTargetingIOInputs inputs) {}
}
