package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static java.util.Map.entry;

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
import frc.lib.team3061.leds.LEDs.ShooterLEDState;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Field2d;
import frc.robot.subsystems.intake.Intake;
import java.util.Map;
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
    64, 57, 53, 45, 43, 41, 38, 36, 35, 33, 32, 31, 29.5, 29.5, 29, 28, 27.5
  };
  private final double[] populationDistances = {
    1.33, 1.63, 1.947, 2.196, 2.47, 2.77, 3.02, 3.32, 3.6, 3.936, 4.206, 4.495, 4.785, 5.083, 5.39,
    5.72, 6.04
  };

  private final double[] passingPopulationDistances = {7.329, 9.649, 11.336};
  private final double[] passingPopulationRealVelocities = {42.0 - 2.0, 50.0 - 2.0, 57.0 - 2.0};

  private final Map<ShootingPosition, Double> positionToAngleMap =
      Map.ofEntries(
          entry(ShootingPosition.PASS, ShooterConstants.PASS_ANGLE),
          entry(ShootingPosition.PODIUM, ShooterConstants.PODIUM_ANGLE),
          entry(ShootingPosition.SUBWOOFER, ShooterConstants.SUBWOOFER_ANGLE),
          entry(ShootingPosition.AMP, ShooterConstants.AMP_ANGLE),
          entry(ShootingPosition.AUTO_SHOT, ShooterConstants.SHOOTER_AUTO_SHOT_ANGLE_DEG),
          entry(ShootingPosition.SOURCE_SIDE_AUTO_1, ShooterConstants.SOURCE_SIDE_AUTO_1_ANGLE),
          entry(ShootingPosition.SOURCE_SIDE_AUTO_2, ShooterConstants.SOURCE_SIDE_AUTO_2_ANGLE),
          entry(ShootingPosition.SOURCE_SIDE_AUTO_3_4, ShooterConstants.SOURCE_SIDE_AUTO_3_4_ANGLE),
          entry(ShootingPosition.STORAGE, ShooterConstants.SHOOTER_STORAGE_ANGLE),
          entry(ShootingPosition.AMP_SIDE_AUTO_1, ShooterConstants.AMP_SIDE_AUTO_1_ANGLE),
          entry(ShootingPosition.AMP_SIDE_AUTO_2, ShooterConstants.AMP_SIDE_AUTO_2_ANGLE),
          entry(ShootingPosition.AMP_SIDE_AUTO_3, ShooterConstants.AMP_SIDE_AUTO_3_ANGLE),
          entry(ShootingPosition.AMP_SIDE_AUTO_4, ShooterConstants.AMP_SIDE_AUTO_4_ANGLE),
          entry(ShootingPosition.AMP_SIDE_AUTO_5, ShooterConstants.AMP_SIDE_AUTO_5_ANGLE),
          entry(ShootingPosition.AMP_SIDE_AUTO_6, ShooterConstants.AMP_SIDE_AUTO_6_ANGLE),
          entry(ShootingPosition.AMP_FAR_SIDE_AUTO_1, ShooterConstants.AMP_FAR_SIDE_AUTO_1_ANGLE));

  @java.lang.SuppressWarnings({"java:S1104"})
  private class Velocity {
    public double top;
    public double bottom;

    public Velocity(double top, double bottom) {
      this.top = top;
      this.bottom = bottom;
    }
  }

  private final Map<ShootingPosition, Velocity> positionToVelocityMap =
      Map.ofEntries(
          entry(
              ShootingPosition.PODIUM,
              new Velocity(
                  ShooterConstants.PODIUM_VELOCITY_TOP, ShooterConstants.PODIUM_VELOCITY_BOTTOM)),
          entry(
              ShootingPosition.SUBWOOFER,
              new Velocity(
                  ShooterConstants.SUBWOOFER_VELOCITY_TOP,
                  ShooterConstants.SUBWOOFER_VELOCITY_BOTTOM)),
          entry(
              ShootingPosition.AMP,
              new Velocity(
                  ShooterConstants.AMP_VELOCITY_TOP, ShooterConstants.AMP_VELOCITY_BOTTOM)),
          entry(
              ShootingPosition.AUTO_SHOT,
              new Velocity(
                  ShooterConstants.SHOOTER_AUTO_SHOT_VELOCITY_RPS,
                  ShooterConstants.SHOOTER_AUTO_SHOT_VELOCITY_RPS)),
          entry(
              ShootingPosition.SOURCE_SIDE_AUTO_1,
              new Velocity(
                  ShooterConstants.SOURCE_SIDE_AUTO_1_VELOCITY,
                  ShooterConstants.SOURCE_SIDE_AUTO_1_VELOCITY)),
          entry(
              ShootingPosition.SOURCE_SIDE_AUTO_2,
              new Velocity(
                  ShooterConstants.SOURCE_SIDE_AUTO_2_VELOCITY,
                  ShooterConstants.SOURCE_SIDE_AUTO_2_VELOCITY)),
          entry(
              ShootingPosition.SOURCE_SIDE_AUTO_3_4,
              new Velocity(
                  ShooterConstants.SOURCE_SIDE_AUTO_3_4_VELOCITY,
                  ShooterConstants.SOURCE_SIDE_AUTO_3_4_VELOCITY)),
          entry(
              ShootingPosition.STORAGE,
              new Velocity(
                  ShooterConstants.SHOOTER_IDLE_VELOCITY, ShooterConstants.SHOOTER_IDLE_VELOCITY)),
          entry(
              ShootingPosition.AMP_SIDE_AUTO_1,
              new Velocity(
                  ShooterConstants.AMP_SIDE_AUTO_1_VELOCITY,
                  ShooterConstants.AMP_SIDE_AUTO_1_VELOCITY)),
          entry(
              ShootingPosition.AMP_SIDE_AUTO_2,
              new Velocity(
                  ShooterConstants.AMP_SIDE_AUTO_2_VELOCITY,
                  ShooterConstants.AMP_SIDE_AUTO_2_VELOCITY)),
          entry(
              ShootingPosition.AMP_SIDE_AUTO_3,
              new Velocity(
                  ShooterConstants.AMP_SIDE_AUTO_3_VELOCITY,
                  ShooterConstants.AMP_SIDE_AUTO_3_VELOCITY)),
          entry(
              ShootingPosition.AMP_SIDE_AUTO_4,
              new Velocity(
                  ShooterConstants.AMP_SIDE_AUTO_4_VELOCITY,
                  ShooterConstants.AMP_SIDE_AUTO_4_VELOCITY)),
          entry(
              ShootingPosition.AMP_SIDE_AUTO_5,
              new Velocity(
                  ShooterConstants.AMP_SIDE_AUTO_5_VELOCITY,
                  ShooterConstants.AMP_SIDE_AUTO_5_VELOCITY)),
          entry(
              ShootingPosition.AMP_SIDE_AUTO_6,
              new Velocity(
                  ShooterConstants.AMP_SIDE_AUTO_6_VELOCITY,
                  ShooterConstants.AMP_SIDE_AUTO_6_VELOCITY)),
          entry(
              ShootingPosition.AMP_FAR_SIDE_AUTO_1,
              new Velocity(
                  ShooterConstants.AMP_FAR_SIDE_AUTO_1_VELOCITY,
                  ShooterConstants.AMP_FAR_SIDE_AUTO_1_VELOCITY)));

  private boolean automatedShooter = true;

  private boolean intakeEnabled = true;
  private final LEDs leds;

  private int topAtSetpointIterationCount = 0;
  private int bottomAtSetpointIterationCount = 0;
  private int angleAtSetpointIterationCount = 0;

  private State state = State.WAITING_FOR_NOTE;
  private State lastState = State.UNINITIALIZED;
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
    UNINITIALIZED {
      @Override
      void execute(Shooter shooter) {
        /* no-op */
      }

      @Override
      void onEnter(Shooter shooter) {
        /* no-op */
      }

      @Override
      void onExit(Shooter shooter) {
        /* no-op */
      }
    },
    WAITING_FOR_NOTE {
      @Override
      void onEnter(Shooter shooter) {
        if (DriverStation.isAutonomousEnabled()) {
          // don't reset the shooting position if we are in autonomous as it has been set at the
          // start of the auto
          shooter.overrideSetpointsForNextShot = true;
        } else {
          shooter.overrideSetpointsForNextShot = false;
          shooter.shootingPosition = ShootingPosition.FIELD;

          // delay retraction of the deflector so it has time to help the note score in the amp
          Commands.sequence(
                  Commands.waitSeconds(shooter.deflectorRetractionDelaySeconds.get()),
                  Commands.runOnce(shooter::retractDeflector))
              .schedule();
        }

        shooter.moveToIntakePosition();
        shooter.setIdleVelocity();
      }

      @Override
      void execute(Shooter shooter) {
        shooter.leds.setShooterLEDState(ShooterLEDState.WAITING_FOR_GAME_PIECE);

        if (shooter.intake.hasNote()) {
          shooter.setState(State.AIMING_AT_SPEAKER);
        }
      }

      @Override
      void onExit(Shooter shooter) {
        /* no-op */
      }
    },
    AIMING_AT_SPEAKER {
      @Override
      void execute(Shooter shooter) {
        shooter.leds.setShooterLEDState(ShooterLEDState.AIMING_AT_SPEAKER);

        if (!shooter.intake.hasNote()) {
          shooter.setState(State.WAITING_FOR_NOTE);
        } else if (shooter.overrideSetpointsForNextShot) {
          shooter.setState(State.PREPARING_TO_SHOOT);
        } else {
          double distanceToSpeaker =
              Field2d.getInstance()
                  .getAllianceSpeakerCenter()
                  .minus(RobotOdometry.getInstance().getEstimatedPosition())
                  .getTranslation()
                  .getNorm();
          shooter.adjustAngle(distanceToSpeaker);
          shooter.setIdleVelocity();
        }
      }

      @Override
      void onEnter(Shooter shooter) {
        /* no-op */
      }

      @Override
      void onExit(Shooter shooter) {
        /* no-op */
      }
    },
    PREPARING_TO_SHOOT {
      @Override
      void execute(Shooter shooter) {
        shooter.leds.setShooterLEDState(ShooterLEDState.AIMING_AT_SPEAKER);

        if (!shooter.intake.hasNote()) {
          shooter.setState(State.WAITING_FOR_NOTE);
        } else {
          double distanceToSpeaker =
              Field2d.getInstance()
                  .getAllianceSpeakerCenter()
                  .minus(RobotOdometry.getInstance().getEstimatedPosition())
                  .getTranslation()
                  .getNorm();
          shooter.adjustAngle(distanceToSpeaker);
          shooter.setRangeVelocity(distanceToSpeaker);
          if (shooter.shootingPosition == ShootingPosition.AMP) {
            shooter.deployDeflector();
          } else {
            shooter.retractDeflector();
          }
        }
      }

      @Override
      void onEnter(Shooter shooter) {
        /* no-op */
      }

      @Override
      void onExit(Shooter shooter) {
        /* no-op */
      }
    };

    abstract void execute(Shooter shooter);

    abstract void onEnter(Shooter shooter);

    abstract void onExit(Shooter shooter);
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

  private void setState(State state) {
    this.state = state;
  }

  private void runAngleStateMachine() {
    if (state != lastState) {
      lastState.onExit(this);
      lastState = state;
      state.onEnter(this);
    }

    state.execute(this);
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
      if (positionToAngleMap.containsKey(shootingPosition)) {
        io.setAngle(positionToAngleMap.get(shootingPosition));
      } else io.setAngle(getAngleForDistance(distanceToSpeaker));
    }
  }

  private void moveToIntakePosition() {
    if (automatedShooter && !DriverStation.isAutonomousEnabled()) {
      io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
    }
  }

  private void setRangeVelocity(double distanceToSpeaker) {
    Logger.recordOutput("Shooter/distanceToSpeaker", distanceToSpeaker);
    double topVelocity;
    double bottomVelocity;

    if (positionToVelocityMap.containsKey(shootingPosition)) {
      topVelocity = positionToVelocityMap.get(shootingPosition).top;
      bottomVelocity = positionToVelocityMap.get(shootingPosition).bottom;
    } else if (shootingPosition == ShootingPosition.PASS) {
      topVelocity = getPassingVelocity();
      bottomVelocity = getPassingVelocity();
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
