package frc.lib.team3061.util;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import java.util.Objects;
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

  private static final double BUFFER_DURATION = 1.5;
  private final TimeInterpolatableBuffer<InterpolationRecord> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(BUFFER_DURATION);

  private static final boolean INCLUDE_VISION_POSE_ESTIMATES = false;

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

    poseBuffer.clear();
  }

  public Pose2d updateWithTime(
      double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    poseBuffer.addSample(
        currentTimeSeconds,
        new InterpolationRecord(
            getEstimatedPosition(), gyroAngle, new SwerveDriveWheelPositions(modulePositions)));
    return this.estimator.updateWithTime(currentTimeSeconds, gyroAngle, modulePositions);
  }

  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      double latencyAdjustmentSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {

    double adjustedTimestamp = timestampSeconds + latencyAdjustmentSeconds;
    if (INCLUDE_VISION_POSE_ESTIMATES) {
      this.estimator.addVisionMeasurement(
          visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);

      if (this.customOdometry != null) {
        this.customOdometry.addVisionMeasurement(
            visionRobotPoseMeters, adjustedTimestamp, visionMeasurementStdDevs);
      }
    }

    // log the difference between the vision pose estimate and the pose estimate corresponding to
    // the same timestamp
    var sample = poseBuffer.getSample(adjustedTimestamp);
    if (!sample.isEmpty()) {
      Pose2d pastPose = sample.get().poseMeters;
      Transform2d diff = pastPose.minus(visionRobotPoseMeters);
      Logger.recordOutput("RobotOdometry/visionPoseDiff", diff);
    }
  }

  public void setCustomOdometry(DrivetrainIOCTRE customOdometry) {
    this.customOdometry = customOdometry;
  }

  public static RobotOdometry getInstance() {
    return robotOdometry;
  }

  // from WPILib's PoseEstimator class

  /**
   * Represents an odometry record. The record contains the inputs provided as well as the pose that
   * was observed based on these inputs, as well as the previous record and its inputs.
   */
  private class InterpolationRecord implements Interpolatable<InterpolationRecord> {
    // The pose observed given the current sensor inputs and the previous pose.
    private final Pose2d poseMeters;

    // The current gyro angle.
    private final Rotation2d gyroAngle;

    // The current encoder readings.
    private final SwerveDriveWheelPositions wheelPositions;

    /**
     * Constructs an Interpolation Record with the specified parameters.
     *
     * @param poseMeters The pose observed given the current sensor inputs and the previous pose.
     * @param gyro The current gyro angle.
     * @param wheelPositions The current encoder readings.
     */
    private InterpolationRecord(
        Pose2d poseMeters, Rotation2d gyro, SwerveDriveWheelPositions wheelPositions) {
      this.poseMeters = poseMeters;
      this.gyroAngle = gyro;
      this.wheelPositions = wheelPositions;
    }

    /**
     * Return the interpolated record. This object is assumed to be the starting position, or lower
     * bound.
     *
     * @param endValue The upper bound, or end.
     * @param t How far between the lower and upper bound we are. This should be bounded in [0, 1].
     * @return The interpolated value.
     */
    @Override
    public InterpolationRecord interpolate(InterpolationRecord endValue, double t) {
      if (t < 0) {
        return this;
      } else if (t >= 1) {
        return endValue;
      } else {
        // Find the new wheel distances.
        var wheelLerp = wheelPositions.interpolate(endValue.wheelPositions, t);

        // Find the new gyro angle.
        var gyroLerp = gyroAngle.interpolate(endValue.gyroAngle, t);

        // Create a twist to represent the change based on the interpolated sensor inputs.
        Twist2d twist =
            RobotConfig.getInstance()
                .getSwerveDriveKinematics()
                .toTwist2d(wheelPositions, wheelLerp);
        twist.dtheta = gyroLerp.minus(gyroAngle).getRadians();

        return new InterpolationRecord(poseMeters.exp(twist), gyroLerp, wheelLerp);
      }
    }

    @Override
    public boolean equals(Object obj) {
      if (this == obj) {
        return true;
      }
      if (!(obj instanceof RobotOdometry.InterpolationRecord)) {
        return false;
      }
      var entry = (RobotOdometry.InterpolationRecord) obj;
      return Objects.equals(gyroAngle, entry.gyroAngle)
          && Objects.equals(wheelPositions, entry.wheelPositions)
          && Objects.equals(poseMeters, entry.poseMeters);
    }

    @Override
    public int hashCode() {
      return Objects.hash(gyroAngle, wheelPositions, poseMeters);
    }
  }
}
