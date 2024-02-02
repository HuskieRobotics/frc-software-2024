package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class Shooter extends SubsystemBase{

  private ShooterIO io;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io){
    this.io = io;

		if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }


  @Override
	public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
  }
  
  public void shoot(double leftMotorVelocity, double rightMotorVelocity){
    io.setShooterMotorLeftVelocity(leftMotorVelocity);
    io.setShooterMotorRightVelocity(rightMotorVelocity);
  }

  public void setDrumMotorVelocity(double velocity){
    io.setDrumMotorVelocity(velocity);
  }

  public void setDunkerMotorVelocity(double velocity){
    io.setDunkerMotorVelocity(velocity);
  }

  public void setAngle(double angle){
    io.setAngle(angle);
  }

  public void getStatorCurrent(){
    io.getStatorCurrent();
  }

  public void getSupplyCurrent(){
    io.getSupplyCurrent();
  }

  public void getVelocity(){
    io.getVelocityRPS();
  }

  public void getSensor(){
    io.getSensor();
  }

  public void getEncoderAngle(){
    io.getEncoderAngle();
  }


    
}
