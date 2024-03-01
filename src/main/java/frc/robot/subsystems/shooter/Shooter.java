package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Field2d;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private Alliance alliance = Alliance.Red;
  private InterpolatingDoubleTreeMap angleTreeMap;
  private InterpolatingDoubleTreeMap veloTreeMap;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  
  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber angle = new TunableNumber("Shooter/Angle", 10.0);

  public BooleanSupplier inStorage = () -> false;

  private boolean autoShooter = false;
  private boolean hasNote = false;
  private boolean isAimed = false;
  private boolean aiming = false;
  private boolean scoringAmp = false;
  private boolean scoringSubwoofer = false;
  private boolean scoringPodium = false;

  private int topAtSetpointIterationCount = 0;
  private int bottomAtSetpointIterationCount = 0;
  private int angleAtSetpointIterationCount = 0;


  public Shooter(ShooterIO io, Drivetrain drivetrain) { // TODO: Add intake reference
    this.io = io;
    this.angleTreeMap = new InterpolatingDoubleTreeMap();
    this.veloTreeMap = new InterpolatingDoubleTreeMap();

    this.autoShooter = false;
    this.hasNote = false;
    this.isAimed = false;
    this.aiming = false;
    this.scoringAmp = false;
    this.scoringSubwoofer = false;
    this.scoringPodium = false;
    this.inStorage = () -> false;

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    this.inStorage = () -> (Math.abs(shooterInputs.angleEncoderAngleDegrees) < ShooterConstants.ANGLE_TOLERANCE);
    // this.hasNote = intake.hasNote();

    if (TESTING) {
      io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
      io.setShooterWheelTopVelocity(topWheelVelocity.get());
      io.setAngle(angle.get());
    }
    else if (autoShooter == false)
    {
      io.setShooterWheelBottomVelocity(SHOOTER_IDLE_VELOCITY);
      io.setShooterWheelTopVelocity(SHOOTER_IDLE_VELOCITY);
    }
    else {
      this.runAngleStateMachine();
      io.setShooterWheelBottomVelocity(SHOOTER_IDLE_VELOCITY);
      io.setShooterWheelTopVelocity(SHOOTER_IDLE_VELOCITY);
    }
  }

  public void updateAlliance(Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  private void runAngleStateMachine() {
    double test =
        Field2d.getInstance()
            .getAllianceSpeakerCenter()
            .minus(RobotOdometry.getInstance().getEstimatedPosition())
            .getTranslation()
            .getNorm();       
    if(hasNote == true){
      if(this.inRange() == true){
        if(scoringPodium == true){
          io.setShooterWheelBottomVelocity(ShooterConstants.PODIUM_VELOCITY);
          io.setShooterWheelTopVelocity(ShooterConstants.PODIUM_VELOCITY);
          io.setAngle(ShooterConstants.PODIUM_ANGLE);
          }
        else if(scoringSubwoofer == true){
          io.setShooterWheelBottomVelocity(ShooterConstants.SUBWOOFER_VELOCITY);
          io.setShooterWheelTopVelocity(ShooterConstants.SUBWOOFER_VELOCITY);
          io.setAngle(ShooterConstants.SUBWOOFER_ANGLE);
          }
        else if(scoringAmp == true){
          io.setShooterWheelBottomVelocity(ShooterConstants.AMP_VELOCITY);
          io.setShooterWheelTopVelocity(ShooterConstants.AMP_VELOCITY);
          io.setAngle(ShooterConstants.AMP_ANGLE);
          }
        else if(aiming == true){
          io.setShooterWheelBottomVelocity(veloTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
          io.setShooterWheelTopVelocity(veloTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
          io.setAngle(angleTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
          }
        else{ // not aiming
          io.setShooterWheelBottomVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
          io.setShooterWheelTopVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
          io.setAngle(angleTreeMap.get(test));
          }
        }
      else{ // not in range
        io.setShooterWheelBottomVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
        io.setShooterWheelTopVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
        io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
        }
      } 
    else{ // no note
      io.setAngle(ShooterConstants.SHOOTER_STORAGE_ANGLE);
      }
    this.goToConstantVelocity();
  }

  public void shoot(double topWheelVelocityRPS, double bottomWheelVelocityRPS) {
    // needs to utilize kicker
    io.setShooterWheelTopVelocity(topWheelVelocityRPS);
    io.setShooterWheelBottomVelocity(bottomWheelVelocityRPS);
  }


  public void setAngle(double angle) {
    io.setAngle(angle);
  }

  public void goToConstantVelocity() {
    io.setShooterWheelTopVelocity(topWheelVelocity.get());
    io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
  }

  public boolean readyToShoot() {
    if(autoShooter == false){
      if(isTopShootAtSetpoint() && isBottomShootAtSetpoint() && isAimed) {
        return true;
      }
      return false;
    }
    else{
      if(isTopShootAtSetpoint() && isBottomShootAtSetpoint() && isAngleAtSetpoint() && isAimed) {
        return true;
      }
      return false;    
    }
  }

  public boolean inRange() {
    if (RobotOdometry.getInstance().getEstimatedPosition().getX() >= Units.inchesToMeters(421.02)
                && this.alliance == Alliance.Blue) {
                  return true;
    }
    else if (RobotOdometry.getInstance().getEstimatedPosition().getX()
                <= Units.inchesToMeters(230.2)
                && this.alliance == Alliance.Red) {
                  return true;
    }
    return false;

    
  }

  public boolean isTopShootAtSetpoint() {
    if (Math.abs(shooterInputs.shootMotorTopVelocityRPS - shooterInputs.shootMotorBottomReferenceVelocityRPS) < VELOCITY_TOLERANCE) {
      topAtSetpointIterationCount++;
      if (topAtSetpointIterationCount >= ShooterConstants.SETPOINTCOUNT) {
        return true;
      }
    } else {
      topAtSetpointIterationCount = 0;
    }
    return false;
  }

  public boolean isBottomShootAtSetpoint() {
    if (Math.abs(shooterInputs.shootMotorBottomVelocityRPS - shooterInputs.shootMotorBottomReferenceVelocityRPS) < VELOCITY_TOLERANCE) {
      bottomAtSetpointIterationCount++;
      if (bottomAtSetpointIterationCount >= ShooterConstants.SETPOINTCOUNT) {
        return true;
      }
    } else {
      bottomAtSetpointIterationCount = 0;
    }
    return false;
  }

  public boolean isAngleAtSetpoint() {
    if (Math.abs(shooterInputs.angleMotorReferenceAngleDegrees - shooterInputs.angleEncoderAngleDegrees) < ANGLE_TOLERANCE) {
      angleAtSetpointIterationCount++;
      if (angleAtSetpointIterationCount >= ShooterConstants.SETPOINTCOUNT) {
        return true;
      }
    } else {
      angleAtSetpointIterationCount = 0;
    }
    return false;
  }

  public boolean inStorage() {
    if(Math.abs(shooterInputs.angleEncoderAngleDegrees) < ANGLE_TOLERANCE){
      return true;
    }
    return false;
  }
}
