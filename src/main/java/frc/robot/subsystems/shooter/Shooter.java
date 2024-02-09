package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private ShooterIO io;
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
