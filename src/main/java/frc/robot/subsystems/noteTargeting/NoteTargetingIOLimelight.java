package frc.robot.subsystems.noteTargeting;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NoteTargetingIOLimelight implements NoteTargetingIO {
  private String name;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;

  public NoteTargetingIOLimelight(String paramName) {
    if (paramName.equals("test")) {
      this.name = "test";
    } else {
      this.name = paramName;
      table = NetworkTableInstance.getDefault().getTable(name);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
    }
  }

  public void updateInputs(NoteTargetingIOInputs inputs) {
    if (name.equals("test")) {
      inputs.x = 0.0;
      inputs.y = 0.0;
      inputs.a = 0.0;
      inputs.hasTarget = false;
    } else {
      inputs.x = tx.getDouble(0.0);
      inputs.y = ty.getDouble(0.0);
      inputs.a = ta.getDouble(0.0);
      inputs.hasTarget = ta.getDouble(0.0) > 0;
    }
  }
}
