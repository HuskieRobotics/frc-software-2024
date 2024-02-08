package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.subsystem.SubsystemIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Climber extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  // private final TunableNumber leftMotorPower = new TunableNumber("Climber/leftPower", 0.0);
  // private final TunableNumber leftMotorCurrent = new TunableNumber("Climber/leftCurrent", 0.0);
  // private final TunableNumber leftMotorPosition = new TunableNumber("Climber/leftPosition", 0.0);

  // private final TunableNumber rightMotorPower = new TunableNumber("Climber/rightPower", 0.0);
  // private final TunableNumber rightMotorCurrent = new TunableNumber("Climber/rightCurrent", 0.0);
  // private final TunableNumber rightMotorPosition = new TunableNumber("Climber/rightPosition", 0.0);

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

  // FIXME: Update inputs needs to be fixed, currently have no idea what to do
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
   * Set the motor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the motor to
   */
  public void setLeftMotorPower(double power) {
    io.setLeftMotorPower(power);
  }

  /**
   * Set the motor current to the specified value in amps.
   *
   * @param power the current to set the motor to in amps
   */
  public void setLeftMotorCurrent(double power) {
    io.setLeftMotorCurrent(power);
  }

  /**
   * Set the motor position to the specified value in degrees.
   *
   * @param position the position to set the motor to in degrees
   */
  public void seLeftMotorPosition(double position) {
    io.setLeftMotorPosition(position, POSITION_FEEDFORWARD);
  }

  // Methods for right motor below
  public void setRightMotorPower(double power) {
    io.setRightMotorPower(power);
  }

  public void setRightMotorCurrent(double power) {
    io.setRightMotorCurrent(power);
  }

  public void setRightMotorPosition(double position) {
    io.setRightMotorPosition(position, POSITION_FEEDFORWARD);
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(
            Commands.run(() -> io.setMotorPower(0.3)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRPM < 2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            CLIMBER,
                            "[System Check] Climber motor not moving as fast as expected",
                            false,
                            true);
                  }
                }),
            Commands.run(() -> io.setMotorPower(-0.2)).withTimeout(1.0),
            Commands.runOnce(
                () -> {
                  if (inputs.velocityRPM > -2.0) {
                    FaultReporter.getInstance()
                        .addFault(
                            CLIMBER,
                            "[System Check] Climber motor moving too slow or in the wrong direction",
                            false,
                            true);
                  }
                }))
        .until(() -> !FaultReporter.getInstance().getFaults(CLIMBER).isEmpty())
        .andThen(Commands.runOnce(() -> io.setMotorPower(0.0)));
  }
}
