package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

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
public class TeleopSwerveAimAtSpeaker extends TeleopSwerve {
  private final Shooter shooter;
  private final Intake intake;

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
  public TeleopSwerveAimAtSpeaker(
      Drivetrain drivetrain,
      Shooter shooter,
      Intake intake,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier) {
    super(
        drivetrain,
        translationXSupplier,
        translationYSupplier,
        calculateTargetRotation(drivetrain));
    this.shooter = shooter;
    this.intake = intake;
  }

  private static Supplier<Rotation2d> calculateTargetRotation(Drivetrain drivetrain) {
    // project the robot pose into the future based on the current velocity
    return () -> {
      Rotation2d targetAngle = drivetrain.getFutureRotationAimedAtSpeaker();
      Logger.recordOutput("TeleopSwerveAimAtSpeaker/targetAngle", targetAngle);
      return targetAngle;
    };
  }

  @Override
  public void initialize() {
    super.initialize();
    this.shooter.prepareToShoot();
  }

  /**
   * This method will be invoked when this command finishes or is interrupted. The drivetrain is
   * left in whatever state it was in when the command finished.
   *
   * @param interrupted true if the command was interrupted by another command being scheduled
   */
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      this.shooter.cancelPrepareToShoot();
    }

    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return this.intake.isShooting();
  }
}
