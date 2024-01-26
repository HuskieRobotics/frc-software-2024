package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import org.littletonrobotics.junction.Logger;

//import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;


public class Shooter extends SubsystemBase{
    
  private final TunableNumber shootKP = new TunableNumber("Shooter/SHOOT_KP", 0.0);
  private final TunableNumber shootKI = new TunableNumber("Shooter/SHOOT_KI", 0.0);
  private final TunableNumber shootKD = new TunableNumber("Shooter/SHOOT_KD", 0.0);

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
  


  public void setAngle(double angle){
    io.setAngle(angle);
  }

  public  void setAppliedCurrent(double current){
    io.setAppliedCurrent(current);
  }

  public void setMotorPower(double power){
    io.setMotorPower(power);
  }

  public void getSensor(){
    io.getSensor();
  }

  public void getEncoderAngle(){
    io.getEncoderAngle();
  }

  public void getVelocity(){
    io.getVelocity();
  }


    
}
