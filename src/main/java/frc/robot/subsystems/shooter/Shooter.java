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
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.ShooterLEDState;
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

  private final TunableNumber testingMode = new TunableNumber("Shooter/TestingMode", 0);

  private final TunableNumber angleManualControlVoltage =
      new TunableNumber("Shooter/ManualControlVoltage", ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE);
  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber pivotAngle = new TunableNumber("Shooter/Angle", 10.0);
  private final double[] populationRealAngles = {64, 54, 44, 40, 37.5, 35, 33, 30, 28};
  private final double[] populationDistances = {
    1.3597 + .06,
    1.9693 + .06,
    2.36 + .06,
    2.72 + .06,
    3.05 + .06,
    3.37 + .06,
    3.7 + .06,
    4.4077 + .06,
    5.0173 + .06
  };

  private boolean autoShooter = true;

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

  // FIXME: consider having 1-2 set distances with fixed angles for auto shots (near subwoofer, near
  // podium) and create a shooter preset for these
  public enum ShootingPosition {
    FIELD,
    AUTO,
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

  public Shooter(ShooterIO io, Intake intake) {
    this.io = io;
    this.intake = intake;
    this.angleTreeMap = new InterpolatingDoubleTreeMap();
    populateAngleMap();

    this.autoShooter = true;

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
      angleTreeMap.put(populationDistances[i], populationRealAngles[i]);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    Logger.recordOutput("Shooter/State", this.state.toString());
    Logger.recordOutput("Shooter/ShootingPosition", this.shootingPosition.toString());
    Logger.recordOutput("Shooter/AngleAutomated", this.autoShooter);
    Logger.recordOutput("Shooter/IntakeAutomated", this.intakeEnabled);
    Logger.recordOutput("Shooter/ScaleDownVelocity", this.scaleDownShooterVelocity);

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
    } else {
      runAngleStateMachine();
    }
  }

  private void runAngleStateMachine() {
    if (state == State.WAITING_FOR_NOTE) {
      if (intake.hasNote()) {
        state = State.AIMING_AT_SPEAKER;
        leds.setShooterLEDState(ShooterLEDState.AIMING_AT_SPEAKER);
      }
      this.moveToIntakePosition();
      this.setIdleVelocity();
      leds.setShooterLEDState(ShooterLEDState.WAITING_FOR_GAME_PIECE);
    } else if (state == State.AIMING_AT_SPEAKER) {
      if (!intake.hasNote()) {
        this.resetToInitialState();
      } else if (overrideSetpointsForNextShot) {
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
        }
      }
    }
  }

  private void resetToInitialState() {
    this.state = State.WAITING_FOR_NOTE;
    this.shootingPosition = ShootingPosition.FIELD;
    this.retractDeflector();
    this.overrideSetpointsForNextShot = DriverStation.isAutonomousEnabled();
  }

  private void setIdleVelocity() {
    double velocity;
    if (intakeEnabled) {
      velocity = ShooterConstants.SHOOTER_IDLE_VELOCITY;
    } else {
      velocity = 0.0;
    }
    io.setShooterWheelBottomVelocity(velocity);
    io.setShooterWheelTopVelocity(velocity);
  }

  private void adjustAngle(double distanceToSpeaker) {
    if (autoShooter) {
      if (shootingPosition == ShootingPosition.PASS) {
        io.setAngle(ShooterConstants.PASS_ANGLE);
      } else if (shootingPosition == ShootingPosition.PODIUM) {
        io.setAngle(ShooterConstants.PODIUM_ANGLE);
      } else if (shootingPosition == ShootingPosition.SUBWOOFER) {
        io.setAngle(ShooterConstants.SUBWOOFER_ANGLE);
      } else if (shootingPosition == ShootingPosition.AMP) {
        io.setAngle(ShooterConstants.AMP_ANGLE);
      } else if (shootingPosition == ShootingPosition.STORAGE) {
        io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
      } else {
        io.setAngle(angleTreeMap.get(distanceToSpeaker));
      }
    }
  }

  private void moveToIntakePosition() {
    if (autoShooter) {
      io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
    }
  }

  private void setRangeVelocity(double distanceToSpeaker) {
    Logger.recordOutput("Shooter/distanceToSpeaker", distanceToSpeaker);
    double topVelocity;
    double bottomVelocity;
    if (shootingPosition == ShootingPosition.PASS) {
      topVelocity = ShooterConstants.PASS_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.PASS_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.PODIUM) {
      topVelocity = ShooterConstants.PODIUM_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.PODIUM_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.SUBWOOFER) {
      topVelocity = ShooterConstants.SUBWOOFER_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.SUBWOOFER_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.AMP) {
      topVelocity = ShooterConstants.AMP_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.AMP_VELOCITY_BOTTOM;
    } else if (shootingPosition == ShootingPosition.STORAGE) {
      topVelocity = ShooterConstants.SHOOTER_IDLE_VELOCITY;
      bottomVelocity = ShooterConstants.SHOOTER_IDLE_VELOCITY;
    } else if (distanceToSpeaker < ShooterConstants.SLOW_TO_MID_VELOCITY_DISTANCE_METERS) {
      topVelocity = ShooterConstants.CLOSE_RANGE_VELOCITY_TOP;
      bottomVelocity = ShooterConstants.CLOSE_RANGE_VELOCITY_BOTTOM;
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
    }

    io.setShooterWheelTopVelocity(topVelocity);
    io.setShooterWheelBottomVelocity(bottomVelocity);
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

  public void enableAutoShooter() {
    this.autoShooter = true;
    this.state = State.WAITING_FOR_NOTE;
  }

  public void disableAutoShooter() {
    this.autoShooter = false;
  }

  public void enableScaleDownShooterVelocity() {
    this.scaleDownShooterVelocity = true;
  }

  public void disableScaleDownShooterVelocity() {
    this.scaleDownShooterVelocity = false;
  }

  public BooleanSupplier getShooterAngleReadySupplier() {
    return () -> {
      return shooterInputs.angleEncoderAngleDegrees < MAX_INTAKE_ANGLE;
    };
  }

  public boolean isShooterReadyToShoot(boolean isAimedAtSpeaker) {
    boolean alignedToShoot =
        isAimedAtSpeaker
            || this.shootingPosition == ShootingPosition.AMP
            || this.shootingPosition == ShootingPosition.AUTO
            || this.shootingPosition == ShootingPosition.PASS
            || DriverStation.isAutonomousEnabled();

    boolean topWheelAtSetpoint = isTopShootAtSetpoint();
    boolean bottomWheelAtSetpoint = isBottomShootAtSetpoint();
    boolean angleAtSetpoint = isAngleAtSetpoint();

    Logger.recordOutput("Shooter/AlignedToShoot", alignedToShoot);
    Logger.recordOutput("Shooter/TopWheelAtSetpoint", topWheelAtSetpoint);
    Logger.recordOutput("Shooter/BottomWheelAtSetpoint", bottomWheelAtSetpoint);
    Logger.recordOutput("Shooter/AngleAtSetpoint", angleAtSetpoint);

    return alignedToShoot
        && topWheelAtSetpoint
        && bottomWheelAtSetpoint
        && (!autoShooter || angleAtSetpoint)
        && this.state == State.PREPARING_TO_SHOOT;
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
    return autoShooter;
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
    io.setDeflectorMotorVoltage(ShooterConstants.DEFLECTOR_DEPLOY_VOLTAGE);
  }

  public void retractDeflector() {
    io.setDeflectorMotorVoltage(ShooterConstants.DEFLECTOR_RETRACT_VOLTAGE);
  }
}
