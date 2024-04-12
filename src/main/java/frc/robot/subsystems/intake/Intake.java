package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.IntakeLEDState;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private static final String SUBSYSTEM_NAME = "INTAKE";

  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LEDs leds;

  private final IntakeMotor[] intakeMotors = {
    IntakeMotor.ROLLER, IntakeMotor.KICKER,
  };

  private BooleanSupplier isShooterAngleReady;

  private IntakeState intakeState;
  private boolean automationEnabled;
  private int intakeAndKickerTimeout;
  private boolean quickShootingEnabled;

  // system tests
  private IntakeState mostRecentIntakeState;
  private boolean checkComplete;

  private final TunableNumber rollerVelocity = new TunableNumber("Intake/Roller Velocity", 0);
  private final TunableNumber kickerVelocity = new TunableNumber("Intake/Kicker Velocity", 0);

  enum IntakeMotor {
    ROLLER,
    KICKER
  }

  enum IntakeState {
    EMPTY,
    NOTE_IN_INTAKE,
    NOTE_IN_INTAKE_AND_KICKER,
    NOTE_IN_BETWEEN_INTAKE_AND_KICKER,
    NOTE_IN_KICKER,
    NOTE_IN_KICKER_AND_SHOOTER,
    NOTE_IN_SHOOTER,
    SHOOTING
  }

  public Intake(IntakeIO io) {
    this.io = io;

    leds = LEDs.getInstance();
    intakeState = IntakeState.EMPTY;
    automationEnabled = true;

    mostRecentIntakeState = null;

    checkComplete = false;

    intakeAndKickerTimeout = 0;

    leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
    this.intakeGamePiece();

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  public void setShooterAngleReady(BooleanSupplier isShooterAngleReady) {
    this.isShooterAngleReady = isShooterAngleReady;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/State", intakeState.toString());
    Logger.recordOutput("Intake/AutomationEnabled", automationEnabled);

    if (TESTING) {
      io.setRollerVelocity(rollerVelocity.get());
      io.setKickerVoltage(kickerVelocity.get());
    } else {
      if (automationEnabled) {
        this.runIntakeStateMachine();
      }
    }
  }

  public void runIntakeStateMachine() {

    if (intakeState == IntakeState.EMPTY) {
      runEmptyState();
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE) {
      runNoteInIntakeState();
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE_AND_KICKER) {
      runNoteInIntakeAndKickerState();
    } else if (intakeState == IntakeState.NOTE_IN_BETWEEN_INTAKE_AND_KICKER) {
      runNoteInBetweenIntakeAndKickerState();
    } else if (intakeState == IntakeState.NOTE_IN_KICKER) {
      runNoteInKickerState();
    } else if (intakeState == IntakeState.NOTE_IN_KICKER_AND_SHOOTER) {
      runNoteInKickerAndShooterState();
    } else if (intakeState == IntakeState.NOTE_IN_SHOOTER) {
      runNoteInShooterState();
    } else if (intakeState == IntakeState.SHOOTING) {
      runShootingState();
    }
  }

  private void runEmptyState() {
    if (isShooterAngleReady.getAsBoolean() || DriverStation.isAutonomousEnabled()) {
      this.intakeGamePiece();
    } else {
      this.repelGamePiece();
    }

    if (inputs.isRollerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_INTAKE;
      leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
      this.transitionGamePiece();
    } else if (inputs.isShooterIRBlocked) {
      intakeState = IntakeState.NOTE_IN_SHOOTER;
      leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
      this.repelGamePiece();
    }
  }

  private void runNoteInIntakeState() {
    if (quickShootingEnabled) {
      intakeState = IntakeState.SHOOTING;
      this.transitionGamePiece();
    } else if (inputs.isKickerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_INTAKE_AND_KICKER;
      this.transitionGamePiece();
    } else if (!inputs.isRollerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_BETWEEN_INTAKE_AND_KICKER;
      intakeAndKickerTimeout = 0;
      this.transitionGamePiece();
    }
  }

  private void runNoteInIntakeAndKickerState() {
    if (!inputs.isRollerIRBlocked) {
      intakeState = IntakeState.NOTE_IN_KICKER;
      this.transitionGamePiece();
      this.repelGamePiece();
    }
  }

  private void runNoteInBetweenIntakeAndKickerState() {
    this.intakeAndKickerTimeout++;

    if (intakeAndKickerTimeout
        > IntakeConstants.IN_BETWEEN_TIMEOUT_SECONDS / Constants.LOOP_PERIOD_SECS) {
      intakeState = IntakeState.EMPTY;
      leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
      this.intakeGamePiece();
    } else if (inputs.isKickerIRBlocked) {
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
    // only transition to the empty state if the note is not in the kicker or the shooter (for amp
    // shots, the note can move such that it isn't detected by the shooter IR sensor)
    if (!this.hasNote()) {
      intakeState = IntakeState.EMPTY;
      leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
      this.turnKickerOff();
      this.intakeGamePiece();
    }
  }

  private void runShootingState() {
    if (!this.hasNote()) {
      intakeState = IntakeState.EMPTY;
      leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
      this.intakeGamePiece();
      this.turnKickerOff();
    } else {
      leds.setIntakeLEDState(IntakeLEDState.SHOOTING);
      this.io.setKickerVoltage(KICKER_SHOOTING_VELOCITY_VOLTAGE);
    }
  }

  public boolean hasNote() {
    return inputs.isKickerIRBlocked || inputs.isShooterIRBlocked;
  }

  public boolean hasNoteForAuto() {
    return inputs.isKickerIRBlocked || inputs.isShooterIRBlocked || inputs.isRollerIRBlocked;
  }

  private void setIntakeState(IntakeState state) {
    this.intakeState = state;
  }

  public void enableQuickShoot() {
    quickShootingEnabled = true;
  }

  public void disableQuickShoot() {
    quickShootingEnabled = false;
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
    this.intakeState = IntakeState.EMPTY;
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
    this.setKickerVelocity(IntakeConstants.KICKER_INTAKING_VELOCITY_RPS);
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
    this.setKickerVelocity(-IntakeConstants.KICKER_INTAKING_VELOCITY_RPS);
  }

  public void setKickerVelocity(double rps) {
    io.setKickerVelocity(rps);
  }

  public void shoot() {
    this.intakeState = IntakeState.SHOOTING;
  }

  public boolean isShooting() {
    return this.intakeState == IntakeState.SHOOTING;
  }
}
