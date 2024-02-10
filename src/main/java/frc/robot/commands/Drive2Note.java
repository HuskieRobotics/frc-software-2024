
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team6328.util.TunableNumber;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Drive2Note extends Command {
  private final Drivetrain drivetrain;
  private final NetworkTable table;
  private final NetworkTableEntry tx;
  private final NetworkTableEntry ty;
  private final NetworkTableEntry ta;
  private static final TunableNumber kP = new TunableNumber("DriveToNote/kP", 0.0);
  private static final TunableNumber kI = new TunableNumber("DriveToNote/kI", 0.0);
  private static final TunableNumber kD = new TunableNumber("DriveToNote/kD", 0.0);
  private final ProfiledPIDController pid;

  public Drive2Note(Drivetrain drivetrain) {

    this.drivetrain = drivetrain;
    table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    pid = new ProfiledPIDController(
                    kP.get(), 
                    kI.get(), 
                    kD.get(), 
                    new TrapezoidProfile.Constraints(thetaMaxVelocity.get(), thetaMaxAcceleration.get()),
                    LOOP_PERIOD_SECS);
  }
  private static final TunableNumber thetaMaxVelocity =
      new TunableNumber(
          "DriveToPose/ThetaMaxVelocity",
          RobotConfig.getInstance().getDriveToPoseTurnMaxVelocity());
  private static final TunableNumber thetaMaxAcceleration =
      new TunableNumber(
          "DriveToPose/ThetaMaxAcceleration",
          RobotConfig.getInstance().getDriveToPoseTurnMaxAcceleration());
  // Rest of the code...

  @Override
  public void initialize() {
    // Add the code that should run at the beginning of the command
    Logger.recordOutput("ActiveCommands/DriveToNote", true);

    //create a pid
    pid.reset(tx.getDouble(0.0));


    
  }

  @Override
  public void execute() {
    // Add the code that should run while the command pis executing
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    //maybe add a check to see if the target is in view and if area is big enough
    double turnVelocity = pid.calculate(x, 0.0);
    Logger.recordOutput("DriveToNote/turnVelocity", turnVelocity);
    drivetrain.drive(0.0, 0.0,turnVelocity,false,false);

  }

  @Override
  public void end(boolean interrupted) {
    // Add any code that should run when the command ends or is interrupted
  }

  @Override
  public boolean isFinished() {
    // Add the condition that determines when the command should end
    // Return true if the command should end, and false otherwise
    return false;
  }
}