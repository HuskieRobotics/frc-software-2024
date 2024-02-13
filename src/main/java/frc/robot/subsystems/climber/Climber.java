package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;

import org.littletonrobotics.junction.Logger;

/**
 * The climber subsystem is responsible for controlling the robot's climber. 
 * It is composed of two motors that are used to control the climber's position and current.
 */
public class Climber extends SubsystemBase {

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private ClimberIO io;

  /**
   * Create a new climber with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Climber(ClimberIO io) {

    this.io = io;

    // Create a Shuffleboard tab for this climber if testing is enabled. Add additional indicators
    // and controls as needed.
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(CLIMBER);
      tab.add(CLIMBER, this);
    }

    FaultReporter.getInstance().registerSystemCheck(CLIMBER, getSystemCheckCommand());
  }

  /**
   * The climber's periodic method needs to update and process the inputs from the hardware
   * interface object.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public void setLeftMotorCurrent(double current) {
    io.setLeftMotorPower(current);
  }

  /**
   * Set the motor position to the specified value in meters.
   *
   * @param position the position to set the motor to in meters
   */
  public void setLeftMotorPosition(double position) {
    io.setLeftMotorPosition(position);
  }

   public void setRightMotorCurrent(double current) {
    io.setRightMotorPower(current);
  }

  /**
   * Set the motor position to the specified value in meters.
   * 
   * @param position the position to set the motor to in meters
   */
  public void setRightMotorPosition(double position) {
    io.setRightMotorPosition(position); // FIXME: Needed info; starting point, arc length. Using these, we can calculate 
    // the height of the arm. Why we would do this is becuase the rate that the arm's height changes is not constant.
  }

  public void setCurrentPositionZero() {
    io.setPositionZero();
  }

  // FIXME: Needs a command check for the right motor. May need to change the commands that are used.
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.run(() -> io.setLeftMotorPower(0.3)).withTimeout(1.0));
  }
}
