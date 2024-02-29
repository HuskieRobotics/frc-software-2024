package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

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
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private Alliance alliance = Alliance.Red;
  private InterpolatingDoubleTreeMap angleTreeMap;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
  
  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber angle = new TunableNumber("Shooter/Angle", 10.0);

  private boolean manualMode = false;
  private boolean hasNote = false;
  private boolean isAimed = false;
  private boolean aiming = false;

  private int topAtSetpointIterationCount = 0;
  private int bottomAtSetpointIterationCount = 0;
  private int angleAtSetpointIterationCount = 0;
  private static final int SETPOINTCOUNT = 0;



  public Shooter(ShooterIO io, Drivetrain drivetrain) { // TODO: Add intake reference
    this.io = io;
    this.angleTreeMap = new InterpolatingDoubleTreeMap();
    this.manualMode = false;
    this.hasNote = false;
    this.isAimed = false;
    this.aiming = false;

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    // this.hasNote = intake.hasNote();

    if (TESTING) {
      io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
      io.setShooterWheelTopVelocity(topWheelVelocity.get());
      io.setAngle(angle.get());
    }
    else if (manualMode == true)
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
        if(aiming == false){
            if (RobotOdometry.getInstance().getEstimatedPosition().getX() >= Units.inchesToMeters(421.02)
                && this.alliance == Alliance.Blue) {
                io.setAngle(angleTreeMap.get(test));
                io.setShooterWheelBottomVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
                io.setShooterWheelTopVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
            } 
            else if (RobotOdometry.getInstance().getEstimatedPosition().getX()
                <= Units.inchesToMeters(230.2)
                && this.alliance == Alliance.Red) {
                io.setAngle(angleTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
                io.setShooterWheelBottomVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
                io.setShooterWheelTopVelocity(ShooterConstants.SHOOTER_IDLE_VELOCITY);
            } 
            else {
                io.setAngle(0);
            }
        }
    }    
    else{
      io.setAngle(0);
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
    if(isTopShootAtSetpoint() && isBottomShootAtSetpoint() && isAngleAtSetpoint() && isAimed) {
      return true;
    }
    return false;
  }

  public boolean isTopShootAtSetpoint() {
    if (Math.abs(shooterInputs.shootMotorTopVelocityRPS - shooterInputs.shootMotorBottomReferenceVelocityRPS) < VELOCITY_TOLERANCE) {
      topAtSetpointIterationCount++;
      if (topAtSetpointIterationCount >= SETPOINTCOUNT) {
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
      if (bottomAtSetpointIterationCount >= SETPOINTCOUNT) {
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
      if (angleAtSetpointIterationCount >= SETPOINTCOUNT) {
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
