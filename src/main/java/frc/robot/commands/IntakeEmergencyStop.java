package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

public class IntakeEmergencyStop extends Command {
  private final Intake intake;

  public IntakeEmergencyStop(Intake intake) {
    this.intake = intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.enableEStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setRightRollerCurrent(0);
    intake.setLeftRollerCurrent(0);
    intake.setDrumCurrent(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // log the end of the command
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.getEStopEnabled();
  }
}
