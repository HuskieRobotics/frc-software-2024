package frc.robot.subsystems.note_targeting;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.lib.team6328.util.TunableNumber;

import java.util.LinkedList;
import java.util.Queue;

public class NoteTargetingIOLimelight implements NoteTargetingIO {
  private String name;
  private NetworkTable table;
  private NetworkTableEntry tx;
  private NetworkTableEntry ty;
  private NetworkTableEntry ta;
  private int noTargetCount;
  private final TunableNumber targetAreaThreshold = new TunableNumber("NoteTargeting/targetAreaThreshold", 0.15);
  private final TunableNumber samplingPeriod = new TunableNumber("NoteTargeting/samplingPeriod", 0.5);
  private final TunableNumber targetPercentage = new TunableNumber("NoteTargeting/targetPercentage", 0.5);
  private Queue<Boolean> targetQueue;

  public NoteTargetingIOLimelight(String paramName) {
    
    if (paramName.equals("test")) {
      this.name = "test";
    } else {
      this.name = paramName;
      table = NetworkTableInstance.getDefault().getTable(name);
      tx = table.getEntry("tx");
      ty = table.getEntry("ty");
      ta = table.getEntry("ta");
      targetQueue = new LinkedList<>(null);

      };
    }

  @Override
  public void updateInputs(NoteTargetingIOInputs inputs) {
    if (name.equals("test")) {
      inputs.x = 0.0;
      inputs.y = 0.0;
      inputs.a = 0.0;
      inputs.hasTarget = false;
    } else {

      inputs.x = -tx.getDouble(0.0);
      inputs.y = ty.getDouble(0.0);
      inputs.a = ta.getDouble(0.0);
      boolean seesTarget = ta.getDouble(0.0) > targetAreaThreshold.get();
      if (seesTarget) {
        //if the target is seen and the queue is full, remove the oldest value
        if (targetQueue.size() >= (int) (samplingPeriod.get()*50)) {
          targetQueue.poll();
        }
        //add the new value to the queue
        targetQueue.add(true);
      } else {
        //if the target is not seen and the queue is full, remove the oldest value
        if (targetQueue.size() >= (int) (samplingPeriod.get()*50)) {
          targetQueue.poll();
        }
        //add the new value to the queue
        targetQueue.add(false);
      }
      //if the percentage of the queue that is true is greater than the target percentage, set hasTarget to true
      //iterate the queue and count the number of true values 
      int trueCount = 0;
      for (Boolean b : targetQueue) {
        if (b) {
          trueCount++;
        }
      }
      //if the percentage of the queue that is true is greater than the target percentage, set hasTarget to true
      inputs.hasTarget = ((double) trueCount / targetQueue.size()) > targetPercentage.get();
      //clear the queue if the target is not had
      if (!inputs.hasTarget) {
        targetQueue.clear();
      }
    }
  }
}
