package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.IntakeLEDState;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private static final String SUBSYSTEM_NAME = "INTAKE";

  private final IntakeIO io;
  private final BooleanSupplier isShooterAngleReady;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LEDs leds;

  private final IntakeMotor[] intakeMotors = {
    IntakeMotor.ROLLER, IntakeMotor.KICKER,
  };

  private IntakeState intakeState;
  private boolean automationEnabled;
  private boolean hasNote;

  // system tests
  private IntakeState mostRecentIntakeState;
  private boolean checkComplete;

  private int intakeAndKickerTimeout;

  enum IntakeMotor {
    ROLLER,
    KICKER
  }

  enum IntakeState {
    EMPTY,
    NOTE_IN_INTAKE,
    IN_BETWEEN_INTAKE_AND_KICKER,
    NOTE_IN_INTAKE_AND_KICKER,
    NOTE_IN_KICKER,
    NOTE_IN_KICKER_AND_SHOOTER,
    NOTE_IN_SHOOTER
  }

  public Intake(IntakeIO io, BooleanSupplier isShooterAngleReady) {
    this.io = io;
    this.isShooterAngleReady = isShooterAngleReady;

    leds = LEDs.getInstance();
    intakeState = IntakeState.EMPTY;
    automationEnabled = true;

    mostRecentIntakeState = null;
    checkComplete = false;

    leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
    this.intakeGamePiece();

    this.intakeAndKickerTimeout = 0;

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", intakeState.toString());

    if (automationEnabled) {
      this.runIntakeStateMachine();
    }
  }

  public void runIntakeStateMachine() {

    if (intakeState == IntakeState.EMPTY) {
      runEmptyState();
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE) {
      runNoteInIntakeState();
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE_AND_KICKER) {
      runNoteInIntakeAndKickerState();
    } else if (intakeState == IntakeState.IN_BETWEEN_INTAKE_AND_KICKER) {
      runInBetweenIntakeAndKickerState();
    } else if (intakeState == IntakeState.NOTE_IN_KICKER) {
      runNoteInKickerState();
    } else if (intakeState == IntakeState.NOTE_IN_KICKER_AND_SHOOTER) {
      runNoteInKickerAndShooterState();
    } else if (intakeState == IntakeState.NOTE_IN_SHOOTER) {
      runNoteInShooterState();
    }
  }

  private void runEmptyState() {
    if (isShooterAngleReady.getAsBoolean()) {
      this.intakeGamePiece();
    } else {
      this.repelGamePiece();
    }

    if (inputs.isRollerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_INTAKE;
      leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
      this.transitionGamePiece();
      hasNote = true;
    } else if (inputs.isShooterIRBlocked) {
      intakeState = IntakeState.NOTE_IN_SHOOTER;
      leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
      this.repelGamePiece();
      hasNote = true;
    }
  }

  private void runNoteInIntakeState() {
    if (inputs.isKickerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_INTAKE_AND_KICKER;
      this.transitionGamePiece();
    } else if (!inputs.isRollerIRBlocked) {
      intakeState = IntakeState.IN_BETWEEN_INTAKE_AND_KICKER;
      this.transitionGamePiece();
    }
  }

  private void runInBetweenIntakeAndKickerState() {
    this.intakeAndKickerTimeout++;

    if (intakeAndKickerTimeout > IntakeConstants.IN_BETWEEN_TIMEOUT_SECONDS * 50) {
      intakeState = IntakeState.EMPTY;
      leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
      this.intakeGamePiece();
      hasNote = false;
    } else if (inputs.isKickerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_KICKER;
      this.transitionGamePiece();
      this.repelGamePiece();
    }
  }

  private void runNoteInIntakeAndKickerState() {
    if (!inputs.isRollerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_KICKER;
      this.transitionGamePiece();
      this.repelGamePiece();
    }
  }

  private void runNoteInKickerState() {
    if (inputs.isShooterIRBlocked) {
      intakeState = IntakeState.NOTE_IN_KICKER_AND_SHOOTER;
      this.turnKickerOff();
    }
  }

  private void runNoteInKickerAndShooterState() {
    if (!inputs.isKickerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_SHOOTER;
    }
  }

  private void runNoteInShooterState() {
    // TODO: Set up access of shooter angle
    if (!inputs.isShooterIRBlocked) {
      intakeState = IntakeState.EMPTY;
      leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
      this.intakeGamePiece();
      hasNote = false;
    }
  }

  public boolean hasNote() {
    return hasNote;
  }

  private void setIntakeState(IntakeState state) {
    this.intakeState = state;
  }

  public void completeCheck() {
    checkComplete = true;
  }

  private void setCheckIncomplete() {
    checkComplete = false;
  }

  private boolean checkComplete() {
    return checkComplete;
  }

  public Command getSystemCheckCommand() {
    /*
     * Order of what will be tested
     * 1. make sure everything works at right velocities
     * 2. then check that all the states it goes through when it gets a note are correct
     *
     */
    return Commands.sequence(
            Commands.runOnce(this::setCheckIncomplete),
            getVelocitiesCheckCommand(),
            getStatesCheckCommand())
        .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
        .andThen(
            Commands.sequence(
                Commands.runOnce(this::turnIntakeOff), Commands.runOnce(this::turnKickerOff)))
        .withName(SUBSYSTEM_NAME + "SystemCheck");
  }

  private Command getVelocitiesCheckCommand() {
    return Commands.parallel(
        Commands.parallel(
            Commands.runOnce(() -> this.setIntakeState(IntakeState.EMPTY)),
            Commands.run(this::intakeGamePiece),
            Commands.run(this::transitionGamePiece)),
        Commands.waitSeconds(1)
            .andThen(
                Commands.runOnce(
                    () -> {
                      for (int i = 0; i < intakeMotors.length; i++) {
                        checkMotorVelocity(intakeMotors[i]);
                      }
                    })));
  }

  private void checkMotorVelocity(IntakeMotor motor) {
    if (motor == IntakeMotor.ROLLER) {
      if (Math.abs(inputs.rollerVelocityRPS - inputs.rollerReferenceVelocityRPS)
          > IntakeConstants.ROLLER_VELOCITY_TOLERANCE) {
        if (Math.abs(inputs.rollerVelocityRPS) > Math.abs(inputs.rollerReferenceVelocityRPS)) {
          FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "[System Check] RRoller Too Fast");
        } else {
          FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "[System Check] Roller Too Slow");
        }
      }
    } else if (motor == IntakeMotor.KICKER
        && inputs.kickerVelocityRPS - inputs.kickerReferenceVelocityRPS
            > IntakeConstants.KICKER_VELOCITY_TOLERANCE) {
      FaultReporter.getInstance().addFault(SUBSYSTEM_NAME, "[System Check] Kicker Too Slow");
    }
  }

  private Command getStatesCheckCommand() {
    IntakeState[] desiredStateSequence = {
      IntakeState.EMPTY,
      IntakeState.NOTE_IN_INTAKE,
      IntakeState.NOTE_IN_INTAKE_AND_KICKER,
      IntakeState.NOTE_IN_KICKER,
      IntakeState.NOTE_IN_KICKER_AND_SHOOTER,
      IntakeState.NOTE_IN_SHOOTER
    };

    ArrayList<IntakeState> actualStateSequence = new ArrayList<>();

    return Commands.parallel(
        Commands.parallel(
            Commands.run(() -> this.setIntakeState(IntakeState.EMPTY)),
            Commands.run(this::intakeGamePiece)),
        Commands.waitSeconds(1)
            .andThen(
                Commands.run(() -> checkStateSequence(actualStateSequence, desiredStateSequence)))
            .until(this::checkComplete));
  }

  private void checkStateSequence(
      ArrayList<IntakeState> actualStateSequence, IntakeState[] desiredStateSequence) {
    if (intakeState != mostRecentIntakeState) {
      actualStateSequence.add(intakeState);
      mostRecentIntakeState = intakeState;
    }

    for (int i = 0; i < actualStateSequence.size(); i++) {
      if (!actualStateSequence.get(i).equals(desiredStateSequence[i])) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "[System Check] Intake State Incorrect, "
                    + "Expected: "
                    + desiredStateSequence[i]
                    + ", "
                    + "Actual: "
                    + actualStateSequence.get(i));
      }
    }

    if (actualStateSequence.size() == desiredStateSequence.length) {
      completeCheck();
    }
  }

  public boolean automationEnabled() {
    return automationEnabled;
  }

  public void enableAutomation() {
    automationEnabled = true;
  }

  public void disableAutomation() {
    automationEnabled = false;
  }

  public void intakeGamePiece() {
    io.setRollerVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
  }

  public void turnIntakeOff() {
    io.setRollerVelocity(0);
  }

  public void transitionGamePiece() {
    this.setKickerVelocity(IntakeConstants.KICKER_VELOCITY_RPS);
  }

  public void turnKickerOff() {
    this.setKickerVelocity(0);
  }

  public void repelGamePiece() {
    io.setRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
  }

  public void outtakeAll() {
    this.repelGamePiece();
    this.outtakeKicker();
  }

  public void outtakeKicker() {
    this.setKickerVelocity(-IntakeConstants.KICKER_VELOCITY_RPS);
  }

  public void setKickerVelocity(double rps) {
    io.setKickerVelocity(rps);
  }

  public void shoot() {
    this.setKickerVelocity(IntakeConstants.KICKER_VELOCITY_RPS);
  }
}
