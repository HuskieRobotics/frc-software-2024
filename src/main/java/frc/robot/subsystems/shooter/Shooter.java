package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainConstants;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Field2d;
import frc.robot.subsystems.intake.Intake;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private Intake intake;
  private Drivetrain drivetrain;
  private InterpolatingDoubleTreeMap angleTreeMap;
  private InterpolatingDoubleTreeMap passingTreeMap;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private final TunableNumber testingMode = new TunableNumber("Shooter/TestingMode", 0);

  private final TunableNumber angleManualControlVoltage =
      new TunableNumber("Shooter/ManualControlVoltage", ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE);
  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber pivotAngle = new TunableNumber("Shooter/Angle", 10.0);
  private final TunableNumber deflectorVoltage = new TunableNumber("Shooter/Deflector Voltage", 0);
  private final TunableNumber futureProjectionSeconds =
      new TunableNumber("Shooter/FutureProjectionSeconds", SHOOTER_AUTO_SHOT_TIME_DELAY_SECS);
  private final TunableNumber deflectorRetractionDelaySeconds =
      new TunableNumber(
          "Shooter/DeflectorRetractionDelaySeconds",
          ShooterConstants.DEFLECTOR_RETRACTION_DELAY_SECS);

  // FIXME: tune on the competition field
  private static final double FIELD_MEASUREMENT_OFFSET = 0.0;
  private final double[] populationRealAngles = {
    66, 59, 54.5, 46.5, 43.5, 42, 38, 36, 35, 33, 32, 31, 29.5, 29.5, 29, 28, 27.5
  };
  private final double[] populationDistances = {
    1.33, 1.63, 1.947, 2.196, 2.47, 2.77, 3.02, 3.32, 3.6, 3.936, 4.206, 4.495, 4.785, 5.083, 5.39,
    5.72, 6.04
  };

  private final double[] passingPopulationDistances = {7.329, 9.649, 11.336};
  private final double[] passingPopulationRealVelocities = {42.0 - 2.0, 50.0 - 2.0, 57.0 - 2.0};

  private boolean automatedShooter = true;

  private boolean intakeEnabled = true;
  private final LEDs leds;

  private int topAtSetpointIterationCount = 0;
  private int bottomAtSetpointIterationCount = 0;
  private int angleAtSetpointIterationCount = 0;

  private State state = State.WAITING_FOR_NOTE;
  private ShootingPosition shootingPosition = ShootingPosition.FIELD;
  private boolean overrideSetpointsForNextShot = false;

  private boolean scaleDownShooterVelocity = false;

  private static final String BUT_IS = " but is ";

  public enum ShootingPosition {
    AUTO_SHOT,
    FIELD,
    SOURCE_SIDE_AUTO_1,
    SOURCE_SIDE_AUTO_2,
    SOURCE_SIDE_AUTO_3_4,
    SPEAKER_AUTO_OUTBOUND,
    SPEAKER_AUTO_INBOUND,
    SPEAKER_AUTO_3,
    SPEAKER_AUTO_4,
    AMP_SIDE_AUTO_1,
    AMP_SIDE_AUTO_2,
    AMP_SIDE_AUTO_3,
    AMP_SIDE_AUTO_4,
    AMP_SIDE_AUTO_5,
    AMP_SIDE_AUTO_6,
    AMP_FAR_SIDE_AUTO_1,
    PASS,
    PODIUM,
    SUBWOOFER,
    AMP,
    STORAGE
  }

  private enum State {
    WAITING_FOR_NOTE,
    AIMING_AT_SPEAKER,
    PREPARING_TO_SHOOT
  }

  public Shooter(ShooterIO io, Intake intake, Drivetrain drivetrain) {
    this.io = io;
    this.intake = intake;
    this.drivetrain = drivetrain;
    this.angleTreeMap = new InterpolatingDoubleTreeMap();
    this.passingTreeMap = new InterpolatingDoubleTreeMap();
    populateAngleMap();
    populatePassingMap();

    this.leds = LEDs.getInstance();

    this.resetToInitialState();

    if (testingMode.get() == 1) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  private void populateAngleMap() {
    for (int i = 0; i < populationRealAngles.length; i++) {
      angleTreeMap.put(populationDistances[i] + FIELD_MEASUREMENT_OFFSET, populationRealAngles[i]);
    }
  }

  private void populatePassingMap() {
    for (int i = 0; i < passingPopulationDistances.length; i++) {
      passingTreeMap.put(
          passingPopulationDistances[i] + FIELD_MEASUREMENT_OFFSET,
          passingPopulationRealVelocities[i]);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    Logger.recordOutput("Shooter/State", this.state.toString());
    Logger.recordOutput("Shooter/ShootingPosition", this.shootingPosition.toString());
    Logger.recordOutput("Shooter/AngleAutomated", this.automatedShooter);
    Logger.recordOutput("Shooter/IntakeAutomated", this.intakeEnabled);
    Logger.recordOutput("Shooter/ScaleDownVelocity", this.scaleDownShooterVelocity);
    Logger.recordOutput("Shooter/DistanceToPassPoint", this.getPassingDistance());
    Logger.recordOutput("Shooter/PassingTarget", Field2d.getInstance().getAlliancePassPose());
    Logger.recordOutput("Shooter/SpeakerPose", Field2d.getInstance().getAllianceSpeakerCenter());
    Logger.recordOutput("Shooter/PassVelocity", this.getPassingVelocity());

    double distanceToSpeaker =
        Field2d.getInstance()
            .getAllianceSpeakerCenter()
            .minus(RobotOdometry.getInstance().getEstimatedPosition())
            .getTranslation()
            .getNorm();
    Logger.recordOutput("Shooter/distanceToSpeaker", distanceToSpeaker);

    if (testingMode.get() == 1) {
      io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
      io.setShooterWheelTopVelocity(topWheelVelocity.get());
      io.setAngle(pivotAngle.get());
      io.setDeflectorMotorVoltage(deflectorVoltage.get());
    } else {
      runAngleStateMachine();
    }
  }

  private void runAngleStateMachine() {
    if (state == State.WAITING_FOR_NOTE) {
      if (intake.hasNote()) {
        state = State.AIMING_AT_SPEAKER;
        leds.requestState(LEDs.States.AIMING_AT_SPEAKER);
      }
      this.moveToIntakePosition();
      this.setIdleVelocity();
      leds.requestState(LEDs.States.WAITING_FOR_GAME_PIECE);
    } else if (state == State.AIMING_AT_SPEAKER) {
      if (!intake.hasNote()) {
        this.resetToInitialState();
      } else if (overrideSetpointsForNextShot) {
        double distanceToSpeaker =
            Field2d.getInstance()
                .getAllianceSpeakerCenter()
                .minus(RobotOdometry.getInstance().getEstimatedPosition())
                .getTranslation()
                .getNorm();
        this.adjustAngle(distanceToSpeaker);
        this.setIdleVelocity();
        state = State.PREPARING_TO_SHOOT;
      } else {
        double distanceToSpeaker =
            Field2d.getInstance()
                .getAllianceSpeakerCenter()
                .minus(RobotOdometry.getInstance().getEstimatedPosition())
                .getTranslation()
                .getNorm();
        this.adjustAngle(distanceToSpeaker);
        this.setIdleVelocity();
      }
    } else if (state == State.PREPARING_TO_SHOOT) {
      if (!intake.hasNote()) {
        this.resetToInitialState();
      } else {
        double distanceToSpeaker =
            Field2d.getInstance()
                .getAllianceSpeakerCenter()
                .minus(RobotOdometry.getInstance().getEstimatedPosition())
                .getTranslation()
                .getNorm();
        this.adjustAngle(distanceToSpeaker);
        this.setRangeVelocity(distanceToSpeaker);
        if (shootingPosition == ShootingPosition.AMP) {
          deployDeflector();
        } else {
          retractDeflector();
        }
      }
    }
  }

  private void resetToInitialState() {
    this.state = State.WAITING_FOR_NOTE;
    if (DriverStation.isAutonomousEnabled()) {
      // don't reset the shooting position if we are in autonomous as it has been set at the start
      // of the auto
      this.overrideSetpointsForNextShot = true;
    } else {
      this.overrideSetpointsForNextShot = false;
      this.shootingPosition = ShootingPosition.FIELD;
      Commands.sequence(
              Commands.waitSeconds(deflectorRetractionDelaySeconds.get()),
              Commands.runOnce(this::retractDeflector))
          .schedule();
    }
  }

  private void setIdleVelocity() {
    // don't idle the shooter wheels in autonomous
    if (!DriverStation.isAutonomous()) {
      if (intakeEnabled && !Constants.DEMO_MODE) {
        io.setShooterWheelBottomVelocity(SHOOTER_IDLE_VELOCITY);
        io.setShooterWheelTopVelocity(SHOOTER_IDLE_VELOCITY);
      } else {
        io.setShooterWheelBottomVelocity(0.0);
        io.setShooterWheelTopVelocity(0.0);
      }
    }
  }

  private double getAngleForDistance(double distanceToSpeaker) {
    if (USE_MATHEMATICAL_MODEL) {
      return 56.599 * (Math.atan(1.651 / (0.6395 * distanceToSpeaker)));
    } else {
      return angleTreeMap.get(distanceToSpeaker);
    }
  }

  private void adjustAngle(double distanceToSpeaker) {
    if (automatedShooter) {
      io.setAngle(getTargetAngle(distanceToSpeaker));
    }
  }

  private double getTargetAngle(double distanceToSpeaker) {
    if (shootingPosition == ShootingPosition.PASS) {
      return ShooterConstants.PASS_ANGLE;
    } else if (shootingPosition == ShootingPosition.PODIUM) {
      return ShooterConstants.PODIUM_ANGLE;
    } else if (shootingPosition == ShootingPosition.SUBWOOFER) {
      return ShooterConstants.SUBWOOFER_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP) {
      return ShooterConstants.AMP_ANGLE;
    } else if (shootingPosition == ShootingPosition.AUTO_SHOT) {
      return ShooterConstants.SHOOTER_AUTO_SHOT_ANGLE_DEG;
    } else if (shootingPosition == ShootingPosition.SOURCE_SIDE_AUTO_1) {
      return ShooterConstants.SOURCE_SIDE_AUTO_1_ANGLE;
    } else if (shootingPosition == ShootingPosition.SOURCE_SIDE_AUTO_2) {
      return ShooterConstants.SOURCE_SIDE_AUTO_2_ANGLE;
    } else if (shootingPosition == ShootingPosition.SOURCE_SIDE_AUTO_3_4) {
      return ShooterConstants.SOURCE_SIDE_AUTO_3_4_ANGLE;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_OUTBOUND) {
      return ShooterConstants.SPEAKER_AUTO_OUTBOUND_ANGLE;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_INBOUND) {
      return ShooterConstants.SPEAKER_AUTO_INBOUND_ANGLE;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_3) {
      return ShooterConstants.SPEAKER_AUTO_3_ANGLE;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_4) {
      return ShooterConstants.SPEAKER_AUTO_4_ANGLE;
    } else if (shootingPosition == ShootingPosition.STORAGE) {
      return ShooterConstants.SHOOTER_STORAGE_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_1) {
      return ShooterConstants.AMP_SIDE_AUTO_1_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_2) {
      return ShooterConstants.AMP_SIDE_AUTO_2_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_3) {
      return ShooterConstants.AMP_SIDE_AUTO_3_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_4) {
      return ShooterConstants.AMP_SIDE_AUTO_4_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_5) {
      return ShooterConstants.AMP_SIDE_AUTO_5_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_6) {
      return ShooterConstants.AMP_SIDE_AUTO_6_ANGLE;
    } else if (shootingPosition == ShootingPosition.AMP_FAR_SIDE_AUTO_1) {
      return ShooterConstants.AMP_FAR_SIDE_AUTO_1_ANGLE;
    } else {
      return getAngleForDistance(distanceToSpeaker);
    }
  }

  private void moveToIntakePosition() {
    if (automatedShooter) {
      if (DriverStation.isAutonomousEnabled()) {
        double distanceToSpeaker =
            Field2d.getInstance()
                .getAllianceSpeakerCenter()
                .minus(RobotOdometry.getInstance().getEstimatedPosition())
                .getTranslation()
                .getNorm();
        double angle =
            Math.min(getTargetAngle(distanceToSpeaker), ShooterConstants.MAX_INTAKE_ANGLE);
        io.setAngle(angle);
      } else {
        io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
      }
    }
  }

  private void setRangeVelocity(double distanceToSpeaker) {
    Logger.recordOutput("Shooter/distanceToSpeaker", distanceToSpeaker);
    double topVelocity;
    double bottomVelocity;
    if (shootingPosition == ShootingPosition.PASS) {
      topVelocity = getPassingVelocity();
      bottomVelocity = getPassingVelocity();
    } else if (shootingPosition == ShootingPosition.PODIUM) {
      topVelocity = ShooterConstants.PODIUM_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.PODIUM_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.SUBWOOFER) {
      topVelocity = ShooterConstants.SUBWOOFER_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.SUBWOOFER_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.AMP) {
      topVelocity = ShooterConstants.AMP_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.AMP_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.AUTO_SHOT) {
      topVelocity = ShooterConstants.SHOOTER_AUTO_SHOT_VELOCITY_RPS;
      bottomVelocity = ShooterConstants.SHOOTER_AUTO_SHOT_VELOCITY_RPS;
    } else if (shootingPosition == ShootingPosition.SOURCE_SIDE_AUTO_1) {
      topVelocity = ShooterConstants.SOURCE_SIDE_AUTO_1_VELOCITY;
      bottomVelocity = ShooterConstants.SOURCE_SIDE_AUTO_1_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SOURCE_SIDE_AUTO_2) {
      topVelocity = ShooterConstants.SOURCE_SIDE_AUTO_2_VELOCITY;
      bottomVelocity = ShooterConstants.SOURCE_SIDE_AUTO_2_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SOURCE_SIDE_AUTO_3_4) {
      topVelocity = ShooterConstants.SOURCE_SIDE_AUTO_3_4_VELOCITY;
      bottomVelocity = ShooterConstants.SOURCE_SIDE_AUTO_3_4_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_OUTBOUND) {
      topVelocity = ShooterConstants.SPEAKER_AUTO_OUTBOUND_VELOCITY;
      bottomVelocity = ShooterConstants.SPEAKER_AUTO_OUTBOUND_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_INBOUND) {
      topVelocity = ShooterConstants.SPEAKER_AUTO_INBOUND_VELOCITY;
      bottomVelocity = ShooterConstants.SPEAKER_AUTO_INBOUND_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_3) {
      topVelocity = ShooterConstants.SPEAKER_AUTO_3_VELOCITY;
      bottomVelocity = ShooterConstants.SPEAKER_AUTO_3_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SPEAKER_AUTO_4) {
      topVelocity = ShooterConstants.SPEAKER_AUTO_4_VELOCITY;
      bottomVelocity = ShooterConstants.SPEAKER_AUTO_4_VELOCITY;
    } else if (shootingPosition == ShootingPosition.STORAGE) {
      topVelocity = ShooterConstants.SHOOTER_IDLE_VELOCITY;
      bottomVelocity = ShooterConstants.SHOOTER_IDLE_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_1) {
      topVelocity = ShooterConstants.AMP_SIDE_AUTO_1_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_SIDE_AUTO_1_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_2) {
      topVelocity = ShooterConstants.AMP_SIDE_AUTO_2_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_SIDE_AUTO_2_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_3) {
      topVelocity = ShooterConstants.AMP_SIDE_AUTO_3_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_SIDE_AUTO_3_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_4) {
      topVelocity = ShooterConstants.AMP_SIDE_AUTO_4_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_SIDE_AUTO_4_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_5) {
      topVelocity = ShooterConstants.AMP_SIDE_AUTO_5_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_SIDE_AUTO_5_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_SIDE_AUTO_6) {
      topVelocity = ShooterConstants.AMP_SIDE_AUTO_6_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_SIDE_AUTO_6_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP_FAR_SIDE_AUTO_1) {
      topVelocity = ShooterConstants.AMP_FAR_SIDE_AUTO_1_VELOCITY;
      bottomVelocity = ShooterConstants.AMP_FAR_SIDE_AUTO_1_VELOCITY;
    } else if (DriverStation.isAutonomousEnabled()) {
      return;
    } else if (distanceToSpeaker < ShooterConstants.SUBWOOFER_TO_NEAR_VELOCITY_DISTANCE_METERS) {
      topVelocity = ShooterConstants.SUBWOOFER_RANGE_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.SUBWOOFER_RANGE_VELOCITY_BOTTOM;
    } else if (distanceToSpeaker < ShooterConstants.NEAR_TO_MID_VELOCITY_DISTANCE_METERS) {
      topVelocity = ShooterConstants.NEAR_RANGE_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.NEAR_RANGE_VELOCITY_BOTTOM;
    } else if (distanceToSpeaker < ShooterConstants.MID_TO_FAST_VELOCITY_DISTANCE_METERS) {
      topVelocity = ShooterConstants.MID_RANGE_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.MID_RANGE_VELOCITY_BOTTOM;
    } else {
      topVelocity = ShooterConstants.FAR_RANGE_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.FAR_RANGE_VELOCITY_BOTTOM;
    }

    // if we are testing in the pits, scale down the velocity to safe levels
    if (scaleDownShooterVelocity) {
      topVelocity *= 0.1;
      bottomVelocity *= 0.1;
    } else if (Constants.DEMO_MODE && shootingPosition != ShootingPosition.AMP) {
      topVelocity *= 0.6;
      bottomVelocity *= 0.6;
    }

    io.setShooterWheelTopVelocity(topVelocity);
    io.setShooterWheelBottomVelocity(bottomVelocity);
  }

  private double getPassingVelocity() {
    double distanceToPassPoint =
        Field2d.getInstance()
            .getAlliancePassPose()
            .minus(drivetrain.getFutureRobotPose(DrivetrainConstants.SHOT_DELAY_SECONDS))
            .getTranslation()
            .getNorm();

    return passingTreeMap.get(distanceToPassPoint);
  }

  private double getPassingDistance() {
    return Field2d.getInstance()
        .getAlliancePassPose()
        .minus(drivetrain.getFutureRobotPose(DrivetrainConstants.SHOT_DELAY_SECONDS))
        .getTranslation()
        .getNorm();
  }

  public void setShootingPosition(ShootingPosition position) {
    this.shootingPosition = position;
    this.overrideSetpointsForNextShot = true;
  }

  public void prepareToShoot() {
    // if we have a note, force the state transition
    if (intake.hasNote()) {
      this.state = State.PREPARING_TO_SHOOT;
    }
  }

  public void cancelPrepareToShoot() {
    // if we have a note, force the state transition
    if (intake.hasNote()) {
      this.state = State.AIMING_AT_SPEAKER;
    }
  }

  public void enableAutomatedShooter() {
    this.automatedShooter = true;
    this.state = State.WAITING_FOR_NOTE;
  }

  public void disableAutomatedShooter() {
    this.automatedShooter = false;
  }

  public void enableScaleDownShooterVelocity() {
    this.scaleDownShooterVelocity = true;
  }

  public void disableScaleDownShooterVelocity() {
    this.scaleDownShooterVelocity = false;
  }

  public BooleanSupplier getShooterAngleReadySupplier() {
    return () -> shooterInputs.angleEncoderAngleDegrees < MAX_INTAKE_ANGLE;
  }

  public boolean isShooterReadyToShoot(boolean isAimedAtSpeaker) {
    boolean alignedToShoot =
        isAimedAtSpeaker
            || this.shootingPosition == ShootingPosition.AMP
            || this.shootingPosition == ShootingPosition.PASS
            || Constants.DEMO_MODE;

    boolean topWheelAtSetpoint = isTopShootAtSetpoint();
    boolean bottomWheelAtSetpoint = isBottomShootAtSetpoint();
    boolean angleAtSetpoint = isAngleAtSetpoint();

    boolean atShootingDistance = isAtShootingDistance();

    Logger.recordOutput("Shooter/AlignedToShoot", alignedToShoot);
    Logger.recordOutput("Shooter/TopWheelAtSetpoint", topWheelAtSetpoint);
    Logger.recordOutput("Shooter/BottomWheelAtSetpoint", bottomWheelAtSetpoint);
    Logger.recordOutput("Shooter/AngleAtSetpoint", angleAtSetpoint);
    Logger.recordOutput("Shooter/AtShootingDistance", atShootingDistance);

    return alignedToShoot
        && topWheelAtSetpoint
        && bottomWheelAtSetpoint
        && (!automatedShooter || angleAtSetpoint)
        && atShootingDistance
        && this.state == State.PREPARING_TO_SHOOT;
  }

  private boolean isAtShootingDistance() {
    if (this.shootingPosition == ShootingPosition.AUTO_SHOT) {
      double distanceToSpeaker =
          drivetrain.getFutureDistanceToSpeaker(futureProjectionSeconds.get());

      return Math.abs(distanceToSpeaker - ShooterConstants.SHOOTER_AUTO_SHOT_DISTANCE_METERS)
          < ShooterConstants.SHOOTER_AUTO_SHOT_TOLERANCE_METERS;
    } else {
      // if auto shot is disabled, always return true since the distance to the speaker is
      // irrelevant
      return true;
    }
  }

  public boolean isTopShootAtSetpoint() {
    if (Math.abs(
            shooterInputs.shootMotorTopVelocityRPS
                - shooterInputs.shootMotorTopReferenceVelocityRPS)
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

  public boolean isAutomated() {
    return automatedShooter;
  }

  public void setAngleMotorVoltage(double voltage) {
    io.setAngleMotorVoltage(voltage * angleManualControlVoltage.get());
  }

  private Command getSystemCheckCommand() {
    return Commands.sequence(
        getPresetCheckCommand(ShootingPosition.AMP),
        getPresetCheckCommand(ShootingPosition.SUBWOOFER),
        getPresetCheckCommand(ShootingPosition.PODIUM),
        getPresetCheckCommand(ShootingPosition.PASS)
            .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
            .andThen(
                Commands.runOnce(
                    () -> {
                      io.setShooterWheelBottomVelocity(0.0);
                      io.setShooterWheelTopVelocity(0.0);
                      io.setAngle(SHOOTER_STORAGE_ANGLE);
                    }))
            .withName(SUBSYSTEM_NAME + "SystemCheck"));
  }

  private Command getPresetCheckCommand(ShootingPosition position) {
    double angle;
    double topVelocity;
    double bottomVelocity;

    if (position == ShootingPosition.SUBWOOFER) {
      angle = SUBWOOFER_ANGLE;
      topVelocity = SUBWOOFER_VELOCITY_TOP;
      bottomVelocity = SUBWOOFER_VELOCITY_BOTTOM;
    } else if (position == ShootingPosition.PODIUM) {
      angle = PODIUM_ANGLE;
      topVelocity = PODIUM_VELOCITY_TOP;
      bottomVelocity = PODIUM_VELOCITY_BOTTOM;
    } else if (position == ShootingPosition.PASS) {
      angle = PASS_ANGLE;
      topVelocity = PASS_VELOCITY_TOP;
      bottomVelocity = PASS_VELOCITY_BOTTOM;
    } else { // ShootingPosition.AMP
      angle = AMP_ANGLE;
      topVelocity = AMP_VELOCITY_TOP;
      bottomVelocity = AMP_VELOCITY_BOTTOM;
    }

    return Commands.sequence(
        Commands.runOnce(() -> this.setShootingPosition(position)),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> this.checkAngle(angle)),
        Commands.runOnce(() -> this.checkVelocity(topVelocity, bottomVelocity)));
  }

  private void checkAngle(double degrees) {
    if (Math.abs(this.shooterInputs.angleEncoderAngleDegrees - degrees) > ANGLE_TOLERANCE_DEGREES) {
      if (Math.abs(degrees) - Math.abs(this.shooterInputs.angleEncoderAngleDegrees) > 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Shooter angle is too low, should be "
                    + degrees
                    + BUT_IS
                    + this.shooterInputs.angleEncoderAngleDegrees);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Shooter angle is too high, should be "
                    + degrees
                    + BUT_IS
                    + this.shooterInputs.angleEncoderAngleDegrees);
      }
    }
  }

  private void checkVelocity(double topVelocity, double bottomVelocity) {
    // check bottom motor
    if (Math.abs(this.shooterInputs.shootMotorBottomVelocityRPS - bottomVelocity)
        > VELOCITY_TOLERANCE) {
      if (Math.abs(this.shooterInputs.shootMotorBottomVelocityRPS) - Math.abs(bottomVelocity) < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Bottom shooter wheel velocity is too low, should be "
                    + bottomVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorBottomVelocityRPS);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Bottom shooter wheel velocity is too high, should be "
                    + bottomVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorBottomVelocityRPS);
      }
    }
    // check top motor
    if (Math.abs(this.shooterInputs.shootMotorTopVelocityRPS - topVelocity) > VELOCITY_TOLERANCE) {
      if (Math.abs(this.shooterInputs.shootMotorTopVelocityRPS) - Math.abs(topVelocity) < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Top shooter wheel velocity is too low, should be "
                    + topVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorTopVelocityRPS);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Top shooter wheel velocity is too high, should be "
                    + topVelocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorTopVelocityRPS);
      }
    }
  }

  public void intakeEnabled() {
    this.intakeEnabled = true;
  }

  public void intakeDisabled() {
    this.intakeEnabled = false;
  }

  public boolean getCoastEnableOverride() {
    return shooterInputs.coastMode;
  }

  public void setCoastModeOverride(boolean coast) {
    if (DriverStation.isDisabled()) {
      io.setCoastMode(coast);
    }
  }

  public void deployDeflector() {
    if (ShooterConstants.DEFLECTOR_ENABLED) {
      io.setDeflectorMotorVoltage(ShooterConstants.DEFLECTOR_DEPLOY_VOLTAGE);
    } else {
      io.setDeflectorMotorVoltage(0.0);
    }
  }

  public void retractDeflector() {
    if (ShooterConstants.DEFLECTOR_ENABLED) {
      io.setDeflectorMotorVoltage(ShooterConstants.DEFLECTOR_RETRACT_VOLTAGE);
    } else {
      io.setDeflectorMotorVoltage(0.0);
    }
  }
}
