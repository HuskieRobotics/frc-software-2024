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
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Climber extends SubsystemBase {

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();
  private ClimberIO io;

  /**
   * Create a new subsystem with its associated hardware interface object.
   *
   * @param io the hardware interface object for this subsystem
   */
  public Climber(ClimberIO io) {

    this.io = io;

    // Create a Shuffleboard tab for this subsystem if testing is enabled. Add additional indicators
    // and controls as needed.
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(CLIMBER);
      tab.add(CLIMBER, this);
    }

    FaultReporter.getInstance().registerSystemCheck(CLIMBER, getSystemCheckCommand());
  }

  /**
   * The subsystem's periodic method needs to update and process the inputs from the hardware
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
  public void setLeftMotorCurrent(double power) {
    io.setLeftMotorPower(power);
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public void setLeftMotorPosition(double position) {
    io.setLeftMotorPosition(position);
  }

   public void setRightMotorCurrent(double power) {
    io.setRightMotorPower(power);
  }

  public void setRightMotorPosition(double position) {
    io.setRightMotorPosition(position);
  }

  // FIXME: Needs a command check for the right motor. May need to change the commands that are used.
  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.run(() -> io.setLeftMotorPower(0.3)).withTimeout(1.0));
  }
}
