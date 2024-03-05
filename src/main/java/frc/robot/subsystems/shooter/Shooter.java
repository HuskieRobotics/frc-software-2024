package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Field2d;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private Intake intake;
  private InterpolatingDoubleTreeMap angleTreeMap;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber angle = new TunableNumber("Shooter/Angle", 10.0);

  private boolean autoShooter = true;

  private int topAtSetpointIterationCount = 0;
  private int bottomAtSetpointIterationCount = 0;
  private int angleAtSetpointIterationCount = 0;

  private ShooterState state = ShooterState.HAS_NOTE;

  public enum ShooterState {
    STORAGE,
    HAS_NOTE,
    AIM,
    SCORE_PODIUM,
    SCORE_SUBWOOFER,
    SCORE_AMP
  }

  public Shooter(ShooterIO io, Intake intake) {
    this.io = io;
    this.intake = intake;
    this.angleTreeMap = new InterpolatingDoubleTreeMap();
    populateAngleMap();

    this.autoShooter = true;

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }

  private void populateAngleMap() {
    // FIXME: update after characterization
    for (int i = 0; i < 10; i++) {
      angleTreeMap.put(0.3 * i + 1.3, 60.0 - i * 5.0);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    Logger.recordOutput("Shooter/State", this.state.toString());

    if (TESTING) {
      io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
      io.setShooterWheelTopVelocity(topWheelVelocity.get());
      io.setAngle(angle.get());
    } else {
      runAngleStateMachine();
    }
  }

  private void runAngleStateMachine() {

    if (intake.hasNote()) {
      double distanceToSpeaker =
          Field2d.getInstance()
              .getAllianceSpeakerCenter()
              .minus(RobotOdometry.getInstance().getEstimatedPosition())
              .getTranslation()
              .getNorm();

      if (state == ShooterState.SCORE_PODIUM) { // 1
        io.setShooterWheelBottomVelocity(ShooterConstants.PODIUM_VELOCITY);
        io.setShooterWheelTopVelocity(ShooterConstants.PODIUM_VELOCITY);
        if (autoShooter) io.setAngle(ShooterConstants.PODIUM_ANGLE);
      } else if (state == ShooterState.SCORE_SUBWOOFER) { // 2
        io.setShooterWheelBottomVelocity(ShooterConstants.SUBWOOFER_VELOCITY);
        io.setShooterWheelTopVelocity(ShooterConstants.SUBWOOFER_VELOCITY);
        if (autoShooter) io.setAngle(ShooterConstants.SUBWOOFER_ANGLE);
      } else if (state == ShooterState.SCORE_AMP) { // 3
        io.setShooterWheelBottomVelocity(ShooterConstants.AMP_VELOCITY);
        io.setShooterWheelTopVelocity(ShooterConstants.AMP_VELOCITY);
        if (autoShooter) io.setAngle(ShooterConstants.AMP_ANGLE);
      } else if (state == ShooterState.AIM && autoShooter) { // 4
        setRangeVelocity(distanceToSpeaker);
        if (autoShooter) io.setAngle(angleTreeMap.get(distanceToSpeaker));
      } else if (state == ShooterState.STORAGE && autoShooter) { // 5
        this.goToIdleVelocity();
        io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
      } else if (state == ShooterState.HAS_NOTE && autoShooter) { // not aiming 5
        this.goToIdleVelocity();
        io.setAngle(angleTreeMap.get(distanceToSpeaker));
      }
    } else if (autoShooter) { // no note 0
      this.goToIdleVelocity();
      io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
    } else {
      // if automation is disabled, stop the shooter wheels so that we don't accidentally
      // shoot notes when we manually intake them
      io.setShooterWheelBottomVelocity(0);
      io.setShooterWheelTopVelocity(0);
    }
  }

  private void setRangeVelocity(double distanceToSpeaker) {
    if (distanceToSpeaker < ShooterConstants.VELOCITY_ZONE_SWITCH_DISTANCE) {
      io.setShooterWheelTopVelocity(ShooterConstants.CLOSE_RANGE_VELOCITY);
      io.setShooterWheelBottomVelocity(ShooterConstants.CLOSE_RANGE_VELOCITY);
    } else {
      io.setShooterWheelTopVelocity(ShooterConstants.FAR_RANGE_VELOCITY);
      io.setShooterWheelBottomVelocity(ShooterConstants.FAR_RANGE_VELOCITY);
    }
  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public void enableAutoShooter() {
    this.autoShooter = true;
  }

  public void disableAutoShooter() {
    this.autoShooter = false;
  }

  public void goToIdleVelocity() {
    io.setShooterWheelTopVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
    io.setShooterWheelBottomVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
  }

  public BooleanSupplier getShooterAngleReadySupplier() {
    return this::isAngleAtSetpoint;
  }

  public boolean isShooterReadyToShoot() {
    if (!autoShooter) {
      return isTopShootAtSetpoint() && isBottomShootAtSetpoint();
    } else {
      return isTopShootAtSetpoint() && isBottomShootAtSetpoint() && isAngleAtSetpoint();
    }
  }

  public boolean isTopShootAtSetpoint() {
    if (Math.abs(
            shooterInputs.shootMotorTopVelocityRPS
                - shooterInputs.shootMotorBottomReferenceVelocityRPS)
        < VELOCITY_TOLERANCE) {
      topAtSetpointIterationCount++;
      if (topAtSetpointIterationCount >= ShooterConstants.SET_POINT_COUNT) {
        return true;
      }
    } else {
      topAtSetpointIterationCount = 0;
    }
    return false;
  }

  public boolean isBottomShootAtSetpoint() {
    if (Math.abs(
            shooterInputs.shootMotorBottomVelocityRPS
                - shooterInputs.shootMotorBottomReferenceVelocityRPS)
        < VELOCITY_TOLERANCE) {
      bottomAtSetpointIterationCount++;
      if (bottomAtSetpointIterationCount >= ShooterConstants.SET_POINT_COUNT) {
        return true;
      }
    } else {
      bottomAtSetpointIterationCount = 0;
    }
    return false;
  }

  public boolean isAngleAtSetpoint() {
    if (Math.abs(
            shooterInputs.angleMotorReferenceAngleDegrees - shooterInputs.angleEncoderAngleDegrees)
        < ANGLE_TOLERANCE_DEGREES) {
      angleAtSetpointIterationCount++;
      if (angleAtSetpointIterationCount >= ShooterConstants.SET_POINT_COUNT) {
        return true;
      }
    } else {
      angleAtSetpointIterationCount = 0;
    }
    return false;
  }

  public boolean isAutoShooter() {
    return autoShooter;
  }

  public void setAngleMotorVoltage(double voltage) {
    io.setAngleMotorVoltage(voltage);
  }
}
