package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.IntakeLEDState;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private static final String SUBSYSTEM_NAME = "INTAKE";

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final LEDs leds;

  private final IntakeMotor[] intakeMotors = { 
    IntakeMotor.RIGHT_ROLLER, 
    IntakeMotor.KICKER,
  };

  enum IntakeMotor {
    RIGHT_ROLLER,
    KICKER
  }
  // Intakes, Kicker, and Shooter (the second kicker IR sensor)
  enum IntakeState {
    EMPTY,
    NOTE_IN_INTAKE,
    NOTE_IN_INTAKE_AND_KICKER,
    NOTE_IN_KICKER,
    NOTE_IN_KICKER_AND_SHOOTER,
    NOTE_IN_SHOOTER
  }

  private IntakeState intakeState;
  private IntakeState mostRecentIntakeState;
  private boolean checkComplete;

  private boolean manualOverrideEnabled;

  public Intake(IntakeIO io) {
    this.io = io;
    leds = LEDs.getInstance();
    intakeState = IntakeState.EMPTY;
    mostRecentIntakeState = null;
    checkComplete = false;

    leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
    this.intakeGamePiece();
    
    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
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
      if (inputs.isRollerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_INTAKE;
        leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
        this.transitionGamePiece();
      } else if (inputs.isShooterIRBlocked) {
        intakeState = IntakeState.NOTE_IN_KICKER;
        leds.setIntakeLEDState(IntakeLEDState.HAS_GAME_PIECE);
        this.repelGamePiece();
      }
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE) {
      if (inputs.isKickerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_INTAKE_AND_KICKER;
        this.transitionGamePiece();
      } else if (!inputs.isRollerIRBlocked) {
        intakeState = IntakeState.EMPTY;
        leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
        this.intakeGamePiece();
      }
    } else if (intakeState == IntakeState.NOTE_IN_INTAKE_AND_KICKER) {
      if (!inputs.isRollerIRBlocked) { 
        intakeState = IntakeState.NOTE_IN_KICKER;
        this.transitionGamePiece();
        this.repelGamePiece();
      }
    } else if (intakeState == IntakeState.NOTE_IN_KICKER) {
      if (inputs.isShooterIRBlocked) {
        intakeState = IntakeState.NOTE_IN_KICKER_AND_SHOOTER;
        this.turnKickerOff();
      }
    } else if (intakeState == IntakeState.NOTE_IN_KICKER_AND_SHOOTER) {
      // the only possible next step is that we become empty after shooting
      if (!inputs.isKickerIRBlocked) {
        intakeState = IntakeState.NOTE_IN_SHOOTER;
      }
    } else if (intakeState == IntakeState.NOTE_IN_SHOOTER) {
      // TODO: Set up access of shooter angle
      if (!inputs.isShooterIRBlocked) {
        intakeState = IntakeState.EMPTY;
        leds.setIntakeLEDState(IntakeLEDState.WAITING_FOR_GAME_PIECE);
        this.intakeGamePiece();
      }
    }
  }

  private void setIntakeState(IntakeState state) {
    this.intakeState = state;
  }

  public void completeCheck() {
    checkComplete = true;
  }

  public void setCheckIncomplete() {
    checkComplete = false;
  }

  public boolean checkComplete() {
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
      getStatesCheckCommand()
    ).until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
    .andThen(Commands.sequence(
      Commands.runOnce(this::turnIntakeOff),
      Commands.runOnce(this::turnKickerOff)))
    .withName(SUBSYSTEM_NAME + "SystemCheck");
  }


  private Command getVelocitiesCheckCommand() {
    return Commands.parallel(
      Commands.parallel(
        Commands.runOnce(() -> this.setIntakeState(IntakeState.EMPTY)),
        Commands.run(this::intakeGamePiece),
        Commands.run(this::transitionGamePiece)
      ),
      Commands.waitSeconds(1).andThen(
        Commands.runOnce(() -> {
          for (int i = 0; i < intakeMotors.length; i++) {
            checkMotorVelocity(intakeMotors[i]);
          }
        })
      )
    );
  }

  private void checkMotorVelocity(IntakeMotor motor) {
    if (motor == IntakeMotor.RIGHT_ROLLER) {
      if (Math.abs(inputs.rollerVelocityRPS 
      - inputs.rollerReferenceVelocityRPS) 
      > IntakeConstants.ROLLER_VELOCITY_TOLERANCE) {
        if (Math.abs(inputs.rollerVelocityRPS) >
        Math.abs(inputs.rollerReferenceVelocityRPS)) {
          FaultReporter.getInstance().addFault(
            SUBSYSTEM_NAME,
            "[System Check] RRoller Too Fast"
          );
        } else {
          FaultReporter.getInstance().addFault(
            SUBSYSTEM_NAME,
            "[System Check] Roller Too Slow"
          );
        }
      }
    } else if (motor == IntakeMotor.KICKER) {
      if (inputs.kickerVelocityRPS 
      - inputs.kickerReferenceVelocityRPS 
      > IntakeConstants.KICKER_VELOCITY_TOLERANCE) {
        FaultReporter.getInstance().addFault(
          SUBSYSTEM_NAME,
          "[System Check] Kicker Too Slow"
        );
      }
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
        Commands.run(this::intakeGamePiece)
      ),
      Commands.waitSeconds(1).andThen(
        Commands.run(() -> checkStateSequence(actualStateSequence, desiredStateSequence))
      ).until(this::checkComplete)
    );
  }

  private void checkStateSequence(ArrayList<IntakeState> actualStateSequence, IntakeState[] desiredStateSequence) {
    if (intakeState != mostRecentIntakeState) {
      actualStateSequence.add(intakeState);
      mostRecentIntakeState = intakeState;
    }

    for (int i = 0; i < actualStateSequence.size(); i++) {
      if (!actualStateSequence.get(i).equals(desiredStateSequence[i])) {
        FaultReporter.getInstance().addFault(
          SUBSYSTEM_NAME,
          "[System Check] Intake State Incorrect, "
          + "Expected: " + desiredStateSequence[i] + ", "
          + "Actual: " + actualStateSequence.get(i)
        );
      }
    }

    if (actualStateSequence.size() == desiredStateSequence.length) {
      completeCheck();
    }

  }

  

  public boolean manualOverrideEnabled() {
    return manualOverrideEnabled;
  }

  public boolean runningManualIntake() {
    return manualOverrideEnabled
        && Math.abs(inputs.rollerReferenceVelocityRPS) > 0.01;
  }

  public void enableManualOverride() {
    manualOverrideEnabled = true;
  }

  public void disableManualOverride() {
    manualOverrideEnabled = false;
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

  public void setKickerVelocity(double rps) {
    io.setKickerVelocity(rps);
  }

  public void shoot() {
    this.setKickerVelocity(IntakeConstants.KICKER_VELOCITY_RPS);
  }
}
