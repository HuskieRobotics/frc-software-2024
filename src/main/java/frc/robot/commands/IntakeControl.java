package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.Intake.IntakeStates;

public class IntakeControl extends Command {
    
  private final Intake intake;

  public IntakeControl(Intake intake) {
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
    if (intake.getDrumIRSensor()) {
        intake.setRightRollerCurrent(IntakeConstants.RIGHT_ROLLER_INTAKING_CURRENT);
        intake.setLeftRollerCurrent(IntakeConstants.LEFT_ROLLER_INTAKING_CURRENT);
        intake.setDrumCurrent(IntakeConstants.DRUM_INTAKING_CURRENT);
    } else {
        intake.setRightRollerCurrent(IntakeConstants.RIGHT_ROLLER_REPELLING_CURRENT);
        intake.setLeftRollerCurrent(IntakeConstants.LEFT_ROLLER_REPELLING_CURRENT);
        intake.setDrumCurrent(IntakeConstants.DRUM_REPELLING_CURRENT);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // log the end of the command
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
