package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class RepelGamePiece extends Command {

  private final Intake intake;

  public RepelGamePiece(Intake intake) {
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRightRollerCurrent(IntakeConstants.RIGHT_ROLLER_REPELLING_CURRENT);
    intake.setLeftRollerCurrent(IntakeConstants.LEFT_ROLLER_REPELLING_CURRENT);
    intake.setDrumCurrent(IntakeConstants.DRUM_REPELLING_CURRENT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.getDrumIRSensor();
  }
}
