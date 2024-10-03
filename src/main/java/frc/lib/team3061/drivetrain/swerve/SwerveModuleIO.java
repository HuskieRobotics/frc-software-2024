package frc.lib.team3061.drivetrain.swerve;

import frc.lib.team3061.drivetrain.DrivetrainIO.SignalPair;
import frc.lib.team3061.drivetrain.DrivetrainIO.SwerveIOInputs;

/** Swerve module hardware abstraction interface. */
public interface SwerveModuleIO {

  /** Updates the set of loggable inputs. */
  public default void updateInputs(SwerveIOInputs inputs) {}

  /** Run the drive motor at the specified percentage of full power. */
  public default void setDriveMotorVoltage(double voltage) {}

  /** Run the angle motor at the specified percentage of full power. */
  public default void setAngleMotorVoltage(double voltage) {}

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity) {}

  /** Run the turn motor to the specified angle. */
  public default void setAnglePosition(double degrees) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setAngleBrakeMode(boolean enable) {}

  /**
   * Returns a pair of status signals related to odometry for the drive motor of the swerve module.
   * This can be used to synchronize the gyro and swerve modules to improve the accuracy of pose
   * estimation. The pair must be constructed such that one signal is the rate of change of the
   * other and latency-compensation can be performed.
   *
   * @return the pair of status signals for the drive motor of the swerve module
   */
  public default SignalPair getOdometryDriveSignalPair() {
    return null;
  }

  /**
   * Returns a pair of status signals related to odometry for the steer motor of the swerve module.
   * This can be used to synchronize the gyro and swerve modules to improve the accuracy of pose
   * estimation. The pair must be constructed such that one signal is the rate of change of the
   * other and latency-compensation can be performed.
   *
   * @return the pair of status signals for the steer motor of the swerve module
   */
  public default SignalPair getOdometryAngleSignalPair() {
    return null;
  }
}
