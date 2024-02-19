package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.IntakeLEDState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LEDs leds;

  private int intakeAndDrumTimeout;

  // Intakes, Drum, Kicker
  enum IntakeState {
    EMPTY, // 0, 0, 0
    IN_BETWEEN_DRUM_AND_KICKER,
    IN_BETWEEN_INTAKE_AND_DRUM,
    NOTE_IN_KICKER, // 0, 0, 1
    NOTE_IN_DRUM, // 0, 1, 0
    NOTE_IN_INTAKE, // 1, 0, 0
    NOTE_IN_INTAKE_AND_DRUM, // 1, 1, 0
    EMERGENCY, // 0, 1, 1 or 1, 0, 1 or 1, 1, 1
  }

  private IntakeState intakeState;
  // will update intake state in different areas / methods based on the sensors,
  // but first just running through the logic of what we would do in these cases

  private boolean manualOverrideEnabled;

  public Intake(IntakeIO io) {
    this.io = io;
    leds = LEDs.getInstance();
    intakeState = IntakeState.EMPTY;

    leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
    this.intakeGamePiece();

    
    FaultReporter.getInstance().registerSystemCheck("INTAKE", getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", intakeState.toString());

    this.runIntakeStateMachine();
  }

  public void runIntakeStateMachine() {

    if (intakeState == IntakeState.EMPTY) {
      // if it is empty, then this is the only 1 possible next steps / situations
      //   1. we are truly empty, and waiting for a game piece
      if (inputs.isRightRollerIRBlocked || inputs.isLeftRollerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_INTAKE;
        leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
        if (inputs.isRightRollerIRBlocked) {
          this.repelGamePieceLeft();
        } else {
          this.repelGamePieceRight();
        }
        this.transitionGamePiece();
      } else if (inputs.isKickerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_KICKER;
        this.repelGamePiece();
      }
    } else if (intakeState == IntakeState.IN_BETWEEN_DRUM_AND_KICKER) {
      if (inputs.isKickerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_KICKER;
        this.turnTransitionOff();
        this.turnKickerOff();
      }
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE) {
      // if there is a note in the intake, then there are two possible next steps depending on the
      // robot
      //   1. we get both the drum and the intake to be sensed at the same time
      //   2. we have no sensors sensed as we are in between the intake and the drum
      if (inputs.isDrumIRBlocked) {
        intakeState = IntakeState.NOTE_IN_INTAKE_AND_DRUM;
        this.transitionGamePiece();
      } else if (inputs.isLeftRollerIRBlocked || inputs.isRightRollerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_INTAKE;
      } else {
        // we are now in between intake and drum
        intakeState = IntakeState.IN_BETWEEN_INTAKE_AND_DRUM;

        // set the timeout here
        intakeAndDrumTimeout = 0;
      }
    } else if (intakeState == IntakeState.IN_BETWEEN_INTAKE_AND_DRUM) {
      //  the only possible next step is that we are only in the drum
      //  add a timeout so that if we are here for too long we go back to being empty
      intakeAndDrumTimeout++;

      if (inputs.isDrumIRBlocked) {
        intakeState = IntakeState.NOTE_IN_DRUM;
        this.transitionGamePiece();
      } else if (intakeAndDrumTimeout >= IntakeConstants.IN_BETWEEN_TIMEOUT_SECONDS * 50) {
        intakeState = IntakeState.EMPTY;
        leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
      }
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE_AND_DRUM) {
      // the only possible next step is that we are only in the drum
      //    1. this is when the intake IR stops being detected
      // assuming that we do not need to call transitionGamePiece() again because we already called
      if (!(inputs.isLeftRollerIRBlocked || inputs.isRightRollerIRBlocked)) {
        intakeState = IntakeState.NOTE_IN_DRUM;
        this.repelGamePiece();
      }
    } else if (intakeState == IntakeState.NOTE_IN_DRUM) {
      // the only possible next step is becoming empty between the drum and the kicker
      if (!inputs.isDrumIRBlocked) {
        intakeState = IntakeState.IN_BETWEEN_DRUM_AND_KICKER;
        this.turnTransitionOff();
        this.repelGamePiece();
      }
    } else if (intakeState == IntakeState.NOTE_IN_KICKER) {
      // the only possible next step is that we become empty after shooting
      if (!inputs.isKickerIRBlocked) {
        intakeState = IntakeState.EMPTY;
        leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
        this.intakeGamePiece();
      }
    }
  }

  public Command getSystemCheckCommand() {
    return null;
  }

  public boolean manualOverrideEnabled() {
    return manualOverrideEnabled;
  }

  public boolean runningManualIntake() {
    return manualOverrideEnabled
        && (inputs.leftRollerReferenceVelocityRPS > 0
            || inputs.rightRollerReferenceVelocityRPS > 0);
  }

  public void enableManualOverride() {
    manualOverrideEnabled = true;
  }

  public void disableManualOverride() {
    manualOverrideEnabled = false;
  }

  public void intakeGamePiece() {
    this.intakeGamePieceRight();
    this.intakeGamePieceLeft();
  }

  public void intakeGamePieceRight() {
    this.setRightRollerVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
  }

  public void intakeGamePieceLeft() {
    this.setLeftRollerVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
  }

  public void turnTransitionOff() {
    this.setDrumVelocity(0);
    // Run kicker as well, integrate with shooter
  }

  public void turnRightIntakeOff() {
    this.setRightRollerVelocity(0);
  }

  public void turnLeftIntakeOff() {
    this.setLeftRollerVelocity(0);
  }

  public void turnIntakeOff() {
    this.turnRightIntakeOff();
    this.turnLeftIntakeOff();
  }

  public void transitionGamePiece() {
    this.setDrumVelocity(IntakeConstants.DRUM_VELOCITY_RPS);
    this.setKickerVelocity(IntakeConstants.KICKER_VELOCITY_RPS);
  }

  public void turnKickerOff() {
    this.setKickerVelocity(0);
  }

  public void repelGamePiece() {
    this.setRightRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
    this.setLeftRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
  }

  public void repelGamePieceRight() {
    this.setRightRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
  }

  public void repelGamePieceLeft() {
    this.setLeftRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
  }

  public void setRightRollerVelocity(double rps) {
    io.setRightRollerVelocity(rps);
  }

  public void setLeftRollerVelocity(double rps) {
    io.setLeftRollerVelocity(rps);
  }

  public void setDrumVelocity(double rps) {
    io.setDrumVelocity(rps);
  }

  public void setKickerVelocity(double rps) {
    io.setKickerVelocity(rps);
  }

  public void shoot() {
    this.setKickerVelocity(IntakeConstants.KICKER_VELOCITY_RPS);
  }
}
