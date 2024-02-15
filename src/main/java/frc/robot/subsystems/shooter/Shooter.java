package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
  private Alliance alliance = Alliance.Red;
  private InterpolatingTreeMap<Double, Double> angleTreeMap;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) { // TODO: Add intake reference
    this.io = io;
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    this.runAngleStateMachine();
  }

  public void updateAlliance(Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  private void runAngleStateMachine() {
    if (RobotOdometry.getInstance().getEstimatedPosition().getX() >= Units.inchesToMeters(421.02)&& this.alliance == Alliance.Blue) {
      io.setAngle(angleTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
    } 
    else if (RobotOdometry.getInstance().getEstimatedPosition().getX() <= Units.inchesToMeters(230.2) && this.alliance == Alliance.Red) {
      io.setAngle(angleTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
    }
    else{
      io.setAngle(0);
    }
  }

  public void shoot(double topWheelVelocityRPS, double bottomWheelVelocityRPS) {
    io.setShooterWheelTopVelocity(topWheelVelocityRPS);
    io.setShooterWheelBottomVelocity(bottomWheelVelocityRPS);
  }

  public void setDunkerMotorVelocity(double velocityRPS) {
    io.setDunkerMotorVelocity(velocityRPS);
  }

  public void setAngle(double angle) {
    io.setAngle(angle);
  }
}
