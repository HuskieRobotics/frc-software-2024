package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private ClimberIO io;

  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  private final TunableNumber testingMode = new TunableNumber("Shooter/TestingMode", 0);
  private final TunableNumber climberVoltage = new TunableNumber("Climber/Voltage", 0.0);

  private ClimberState climberState;

  private enum ClimberState {
    IDLE,
    EXTENDING,
    RETRACTING,
    RETRACTING_SLOW
  }

  public Climber(ClimberIO io) {
    this.io = io;
    this.climberState = ClimberState.IDLE;

    if (testingMode.get() == 1) {
      ShuffleboardTab tab = Shuffleboard.getTab(ClimberConstants.SUBSYSTEM_NAME);
      tab.add(ClimberConstants.SUBSYSTEM_NAME, this);
    }
  }

  @Override
  public void periodic() {
    if (testingMode.get() == 0) {
      io.updateInputs(inputs);
      Logger.processInputs("Climber", inputs);

      runClimberStateMachine();
    } else {
      io.setClimberVoltage(climberVoltage.get());
    }

    Logger.recordOutput("Climber/State", this.climberState);
  }

  public void extendClimber() {
    climberState = ClimberState.EXTENDING;
  }

  public void retractClimber() {
    climberState = ClimberState.RETRACTING;
  }

  public void resetClimber() {
    climberState = ClimberState.RETRACTING_SLOW;
  }

  public void zeroClimber() {
    this.climberState = ClimberState.IDLE;
    io.zeroPosition();
  }

  private void runClimberStateMachine() {
    if (climberState == ClimberState.IDLE) {
      io.setClimberVoltage(0.0);
    } else if (climberState == ClimberState.EXTENDING) {
      io.setClimberVoltage(ClimberConstants.EXTENDING_VOLTAGE);

      if (inputs.climberMotorPositionRotations >= ClimberConstants.EXTENDED_POSITION_ROT) {
        climberState = ClimberState.IDLE;
      }
    } else if (climberState == ClimberState.RETRACTING) {
      io.setClimberVoltage(ClimberConstants.RETRACTING_VOLTAGE);
    } else if (climberState == ClimberState.RETRACTING_SLOW) {
      io.setClimberVoltage(ClimberConstants.RETRACTING_VOLTAGE);
    }
  }
}
