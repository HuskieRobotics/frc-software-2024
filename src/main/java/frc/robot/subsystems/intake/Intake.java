package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    // Intakes, Drum, Kicker
    enum IntakeState {
        EMPTY,                   // 0, 0, 0
        NOTE_IN_KICKER,          // 0, 0, 1
        NOTE_IN_DRUM,            // 0, 1, 0
        NOTE_IN_INTAKE,          // 1, 0, 0
        NOTE_IN_INTAKE_AND_DRUM, // 1, 1, 0
        EMERGENCY,               // 0, 1, 1 or 1, 0, 1 or 1, 1, 1
    }

    private IntakeState intakeState;
    // will update intake state in different areas / methods based on the sensors,
    // but first just running through the logic of what we would do in these cases

    private boolean manualOverrideEnabled;
    private boolean runningManualIntake;

    public Intake(IntakeIO io) {
        this.io = io;
        intakeState = IntakeState.EMPTY;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Intake", inputs);

        if (!manualOverrideEnabled()) {
            if (intakeState == IntakeState.EMPTY) {
                this.intakeGamePiece();
            } else if (intakeState == IntakeState.NOTE_IN_INTAKE 
                    || intakeState == IntakeState.NOTE_IN_INTAKE_AND_DRUM) {
                this.intakeGamePiece();
                this.transitionGamePiece();
            } else if (intakeState == IntakeState.NOTE_IN_DRUM) {
                this.transitionGamePiece();
                this.repelGamePiece();
            } else if (intakeState == IntakeState.NOTE_IN_KICKER) {
                this.turnTransitionOff();
            } else if (intakeState == IntakeState.EMERGENCY) {
                this.turnTransitionOff();
                this.turnRightIntakeOff();
                this.turnLeftIntakeOff();
            }
        }
    }

    public void getIntakeSensorState() {
        // get sensor state
        // update intake state
        if (!(inputs.isRightRollerIRBlocked || inputs.isLeftRollerIRBlocked || inputs.isDrumIRBlocked)) {
            intakeState = IntakeState.EMPTY;
        } else if (inputs.isRightRollerIRBlocked && inputs.isLeftRollerIRBlocked && inputs.isDrumIRBlocked) {
            intakeState = IntakeState.EMERGENCY;
        } else if (inputs.isRightRollerIRBlocked && inputs.isDrumIRBlocked) {
            intakeState = IntakeState.NOTE_IN_INTAKE_AND_DRUM;
        }  else if (inputs.isRightRollerIRBlocked || inputs.isLeftRollerIRBlocked) {
            intakeState = IntakeState.NOTE_IN_INTAKE;
        } else if (inputs.isDrumIRBlocked) {
            intakeState = IntakeState.NOTE_IN_DRUM;
        } else if ( true /* access kicker sensor */) {
            intakeState = IntakeState.NOTE_IN_KICKER;
        }
    }

    public boolean getRightRollerIRSensor() {
        return inputs.isRightRollerIRBlocked;
    }

    public boolean getLeftRollerIRSensor() {
        return inputs.isLeftRollerIRBlocked;
    }

    public boolean getDrumIRSensor() {
        return inputs.isDrumIRBlocked;
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
        // Run kicker as well, integrate with shooter
    }

    public void repelGamePiece() {
        this.setRightRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
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
}
