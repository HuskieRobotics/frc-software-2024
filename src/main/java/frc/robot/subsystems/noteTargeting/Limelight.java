package frc.robot.subsystems.noteTargeting;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Limelight extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry tx;
    private final NetworkTableEntry ty;
    private final NetworkTableEntry ta;
    private final String SUBSYSTEM_NAME = "Limelight";




    public Limelight(String name)  {
        table = NetworkTableInstance.getDefault().getTable(name);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        log();
    }
    private void log(){
        Logger.recordOutput(SUBSYSTEM_NAME+"/tx/", tx.getDouble(0.0));
        Logger.recordOutput(SUBSYSTEM_NAME+"/ty/", ty.getDouble(0.0));
        Logger.recordOutput(SUBSYSTEM_NAME+"/ta/", ta.getDouble(0.0));
        Logger.recordOutput(SUBSYSTEM_NAME+"/hasTarget/", hasTarget());
    }

    public Limelight() {
        this("limelight");
    }
    public double getTx() {
        return tx.getDouble(0.0);
    }
    public double getTy() {
        return ty.getDouble(0.0);
    }
    public double getTa() {
        return ta.getDouble(0.0);
    }
    public boolean hasTarget() {

        return ta.getDouble(0.0) > 0;
    }
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //Check if the limelight has a target
        //if it does change led color
        if (hasTarget()) {
            //LEDs.getInstance().setLEDColor();
        }
    }


}
