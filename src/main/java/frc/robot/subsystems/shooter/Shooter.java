package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber pivotAngle = new TunableNumber("Shooter/Angle", 10.0);
  private final double[] populationRealAngles = {60.2, 39.3, 46.1, 38.7, 33, 28.8, 26.8};
  private final double[] populationRobotAngles = {53.17, 33.5, 39, 31.82, 28, 23, 20};
  private final double[] populationDistances = {
    Units.inchesToMeters(53.53),
    Units.inchesToMeters(119.194),
    Units.inchesToMeters(88.53),
    Units.inchesToMeters(113.53),
    Units.inchesToMeters(137.53),
    Units.inchesToMeters(161.53),
    Units.inchesToMeters(185.53)
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

  private Trigger coastModeButton = new Trigger(() -> shooterInputs.coastMode);

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

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }

    FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  private void populateAngleMap() {
    for (int i = 0; i < populationRealAngles.length; i++) {
      angleTreeMap.put(populationDistances[i], populationRobotAngles[i]);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    Logger.recordOutput("Shooter/State", this.state.toString());
    Logger.recordOutput("Shooter/ShootingPosition", this.shootingPosition.toString());

    if (TESTING) {
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
      }
      io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
      this.setIdleVelocity();
      leds.setShooterLEDState(ShooterLEDState.WAITING_FOR_GAME_PIECE);
    } else if (state == State.AIMING_AT_SPEAKER) {
      if (!intake.hasNote()) {
        this.resetToInitialState();
      } else if (overrideSetpointsForNextShot) {
        state = State.PREPARING_TO_SHOOT;
        leds.setShooterLEDState(ShooterLEDState.IS_READY_TO_SHOOT);
      } else {
        double distanceToSpeaker =
            Field2d.getInstance()
                .getAllianceSpeakerCenter()
                .minus(RobotOdometry.getInstance().getEstimatedPosition())
                .getTranslation()
                .getNorm();
        this.adjustAngle(distanceToSpeaker);
        this.setIdleVelocity();
        leds.setShooterLEDState(ShooterLEDState.IS_READY_TO_SHOOT);
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
      }
    }
  }

  private void resetToInitialState() {
    this.state = State.WAITING_FOR_NOTE;
    this.shootingPosition = ShootingPosition.FIELD;
    this.overrideSetpointsForNextShot = false;
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

  private void setRangeVelocity(double distanceToSpeaker) {
    double velocity;
    if (shootingPosition == ShootingPosition.PASS) {
      velocity = ShooterConstants.PASS_VELOCITY;
    } else if (shootingPosition == ShootingPosition.PODIUM) {
      velocity = ShooterConstants.PODIUM_VELOCITY;
    } else if (shootingPosition == ShootingPosition.SUBWOOFER) {
      velocity = ShooterConstants.SUBWOOFER_VELOCITY;
    } else if (shootingPosition == ShootingPosition.AMP) {
      // FIXME: add support for different top and bottom velocities
      velocity = ShooterConstants.AMP_VELOCITY;
    } else if (shootingPosition == ShootingPosition.STORAGE) {
      velocity = ShooterConstants.SHOOTER_IDLE_VELOCITY;
    } else if (distanceToSpeaker < ShooterConstants.VELOCITY_ZONE_SWITCH_DISTANCE) {
      velocity = ShooterConstants.CLOSE_RANGE_VELOCITY;
    } else {
      velocity = ShooterConstants.FAR_RANGE_VELOCITY;
    }

    io.setShooterWheelBottomVelocity(velocity);
    io.setShooterWheelTopVelocity(velocity);
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
  }

  public void disableAutoShooter() {
    this.autoShooter = false;
  }

  public BooleanSupplier getShooterAngleReadySupplier() {
    return this::isAngleAtSetpoint;
  }

  public boolean isShooterReadyToShoot(boolean isAimedAtSpeaker) {
    boolean alignedToShoot =
        isAimedAtSpeaker
            || this.shootingPosition == ShootingPosition.AMP
            || this.shootingPosition == ShootingPosition.AUTO
            || this.shootingPosition == ShootingPosition.PASS;

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
        && (!autoShooter || angleAtSetpoint);
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

  public boolean isAutomated() {
    return autoShooter;
  }

  public void setAngleMotorVoltage(double voltage) {
    io.setAngleMotorVoltage(voltage);
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
    double velocity;

    if (position == ShootingPosition.SUBWOOFER) {
      angle = SUBWOOFER_ANGLE;
      velocity = SUBWOOFER_VELOCITY;
    } else if (position == ShootingPosition.PODIUM) {
      angle = PODIUM_ANGLE;
      velocity = PODIUM_VELOCITY;
    } else if (position == ShootingPosition.PASS) {
      angle = PASS_ANGLE;
      velocity = PASS_VELOCITY;
    } else { // ShootingPosition.AMP
      angle = AMP_ANGLE;
      velocity = AMP_VELOCITY;
    }

    return Commands.sequence(
        Commands.runOnce(() -> this.setShootingPosition(position)),
        Commands.waitSeconds(2.0),
        Commands.runOnce(() -> this.checkAngle(angle)),
        Commands.runOnce(() -> this.checkVelocity(velocity)));
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

  private void checkVelocity(double velocity) {
    // check bottom motor
    if (Math.abs(this.shooterInputs.shootMotorBottomVelocityRPS - velocity) > VELOCITY_TOLERANCE) {
      if (Math.abs(this.shooterInputs.shootMotorBottomVelocityRPS) - Math.abs(velocity) < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Bottom shooter wheel velocity is too low, should be "
                    + velocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorBottomVelocityRPS);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Bottom shooter wheel velocity is too high, should be "
                    + velocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorBottomVelocityRPS);
      }
    }
    // check top motor
    if (Math.abs(this.shooterInputs.shootMotorTopVelocityRPS - velocity) > VELOCITY_TOLERANCE) {
      if (Math.abs(this.shooterInputs.shootMotorTopVelocityRPS) - Math.abs(velocity) < 0) {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Top shooter wheel velocity is too low, should be "
                    + velocity
                    + BUT_IS
                    + this.shooterInputs.shootMotorTopVelocityRPS);
      } else {
        FaultReporter.getInstance()
            .addFault(
                SUBSYSTEM_NAME,
                "Top shooter wheel velocity is too high, should be "
                    + velocity
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
    io.setCoastMode(coast);
  }
}
