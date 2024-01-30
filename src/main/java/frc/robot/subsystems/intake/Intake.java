package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {

    private final IntakeIO io;
    private final IntakeIOInputs inputs = new IntakeIOInputs();

    private boolean manualOverrideEnabled;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
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

    public boolean getManualOverrideEnabled() {
        return manualOverrideEnabled;
    }

    public void intakeGamePiece() {
        this.setRightRollerVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
        this.setLeftRollerVelocity(IntakeConstants.INTAKE_VELOCITY_ROLLERS_RPS);
        this.setDrumVelocity(IntakeConstants.INTAKE_VELOCITY_DRUM_RPS);
    }

    public void repelGamePiece() {
        this.setRightRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
        this.setLeftRollerVelocity(IntakeConstants.REPEL_VELOCITY_ROLLERS_RPS);
        this.setDrumVelocity(IntakeConstants.REPEL_VELOCITY_DRUM_RPS);
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
