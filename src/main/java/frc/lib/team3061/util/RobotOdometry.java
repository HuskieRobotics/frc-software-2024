package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import org.littletonrobotics.junction.Logger;

@java.lang.SuppressWarnings({"java:S6548"})

/**
 * Singleton class for SwerveDrivePoseEstimator that allows it to be shared by subsystems
 * (drivetrain and vision)
 */
public class RobotOdometry {
  private static final RobotOdometry robotOdometry = new RobotOdometry();
  private SwerveDrivePoseEstimator estimator = null;
  private DrivetrainIOCTRE customOdometry = null;
  private SwerveModulePosition[] defaultPositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };

  private RobotOdometry() {
    estimator =
        new SwerveDrivePoseEstimator(
            RobotConfig.getInstance().getSwerveDriveKinematics(),
            new Rotation2d(),
            defaultPositions,
            new Pose2d());
  }

  public Pose2d getEstimatedPosition() {
    // changed to not consider custom odometry at all
    return this.estimator.getEstimatedPosition();
  }

  // new method to get estimated position of the custom odometry (in order to compare with the
  // default estimator)
  public Pose2d getCustomEstimatedPosition() {
    return this.customOdometry != null ? this.customOdometry.getEstimatedPosition() : null;
  }

  public void resetPosition(
      Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d poseMeters) {

    // resetting position on both estimators at the same time since we are considering both in
    // parallel now
    this.estimator.resetPosition(gyroAngle, modulePositions, poseMeters);
    if (this.customOdometry != null)
      this.customOdometry.resetPosition(gyroAngle, modulePositions, poseMeters);
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    // ignore all vision updates in the first stage of tuning
    // if (this.customOdometry == null) {
    //   this.estimator.addVisionMeasurement(
    //   visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    // } else {
    // if (this.customOdometry != null) {
    //   this.customOdometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds,
    // visionMeasurementStdDevs);
    // }

    // log the difference between the vision pose estimate and the current pose estimate; this is
    // most useful if the robot is at rest as we aren't accounting for the latency of the vision
    Pose2d currentPose = this.getCustomEstimatedPosition();
    if (currentPose != null) {
      Transform2d diff = currentPose.minus(visionRobotPoseMeters);
      Logger.recordOutput("RobotOdometry", diff);
    }
  }

  public void setCustomOdometry(DrivetrainIOCTRE customOdometry) {
    this.customOdometry = customOdometry;
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }
}
