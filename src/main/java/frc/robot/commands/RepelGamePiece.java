package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

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
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeGamePiece intakeGamePiece = new IntakeGamePiece(intake);
    intakeGamePiece.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getCurrentIntakeState() == Intake.IntakeStates.DRUMFILLED;
  }
}
