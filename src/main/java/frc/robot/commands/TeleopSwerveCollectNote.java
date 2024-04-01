package frc.robot.commands;

import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.note_targeting.NoteTargeting;
import java.util.function.DoubleSupplier;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive based on the specified
 * values from the controller(s) and enabled features. Configurable deadband and power function are
 * applied to the controller inputs. If the robot isn't in "turbo" mode, the acceleration is limited
 * based on the configurable constraints. This command is designed to be the default command for the
 * drivetrain subsystem.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: never
 *
 * <p>At End: stops the drivetrain
 */
public class TeleopSwerveCollectNote extends TeleopSwerve {
  private final Drivetrain drivetrain;
  private final Intake intake;
  private final NoteTargeting noteTargeting;
  private boolean wasFieldRelative;

  /**
   * Create a new TeleopSwerve command object.
   *
   * @param drivetrain the drivetrain subsystem instructed by this command
   * @param translationXSupplier the supplier of the translation x value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param translationYSupplier the supplier of the translation y value as a percentage of the
   *     maximum velocity as defined by the standard field or robot coordinate system
   * @param rotationSupplier the supplier of the rotation value as a percentage of the maximum
   *     rotational velocity as defined by the standard field or robot coordinate system
   */
  public TeleopSwerveCollectNote(
      Drivetrain drivetrain,
      Intake intake,
      NoteTargeting noteTargeting,
      DoubleSupplier translationXSupplier) {
    super(drivetrain, translationXSupplier, noteTargeting::getAdjustment, () -> 0.0);
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.noteTargeting = noteTargeting;
    this.wasFieldRelative = true;
  }

  @Override
  public void initialize() {
    super.initialize();
    this.wasFieldRelative = drivetrain.getFieldRelative();
    drivetrain.disableFieldRelative();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. The drivetrain is
   * left in whatever state it was in when the command finished.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    if (this.wasFieldRelative) {
      drivetrain.enableFieldRelative();
    } else {
      drivetrain.disableFieldRelative();
    }

    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return this.intake.hasNoteForAuto() || !this.noteTargeting.hasTarget();
  }
}