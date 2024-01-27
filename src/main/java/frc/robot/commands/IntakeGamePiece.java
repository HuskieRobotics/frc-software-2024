package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;

public class IntakeGamePiece extends Command {

  private final Intake intake;

  public IntakeGamePiece(Intake intake) {
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // tell the log the command has started
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRightRollerCurrent(IntakeConstants.ROLLERS_CONTINUOUS_CURRENT_LIMIT);
    intake.setLeftRollerCurrent(IntakeConstants.ROLLERS_CONTINUOUS_CURRENT_LIMIT);
    intake.setDrumCurrent(IntakeConstants.DRUM_CONTINUOUS_CURRENT_LIMIT);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // log the end of the command?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getDrumIRSensor();
  }
}
