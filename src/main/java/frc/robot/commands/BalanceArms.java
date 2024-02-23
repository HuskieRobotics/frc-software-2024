// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.climber.ClimberConstants.KP;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.subsystems.climber.Climber;

import edu.wpi.first.math.controller.PIDController;

/**
 * IDEA: Have arms move to a predetermined position in rotations. Once aligned with chain, retract arms until
 * a current spike is detected one arm, then stop that arm. Do this until both arms have a current spike. 
 * Take the # of rotations and subtract from the predetermined position to get each arms position. 
 * Now that the arms are on the chain, begin retracting the arms at constant speed until the robot is balanced.
 */

public class BalanceArms extends Command {
  // FIXME: Update constants
  private static final double THRESHOLD = 5.0;
  private static final double MAX_VELOCITY_MPS = 0.0;
  private static final double CONSTANT_CLIMBER_CURRENT = 6.0;

  TunableNumber autoKP = new TunableNumber("Climber/AutoBalanceKP", 0.0);
  TunableNumber autoKI = new TunableNumber("Climber/AutoBalanceKI", 0.0);
  TunableNumber autoKD = new TunableNumber("Climber/AutoBalanceKD", 0.0);

  private static double autoBalanceKP = 0.0;
  private static double autoBalanceKI = 0.0;
  private static double autoBalanceKD = 0.0;

  private boolean isLongArms = false;

  private Climber climber;
  private Drivetrain drivetrain;
  private PIDController pidController;

  private enum ClimbingState {
    EXTENDING,
    RETRACTING,
    BALANCED
  }



  /** Creates a new BalanceArms. */
  public BalanceArms(Climber climber, Drivetrain drivetrain) {
    this.climber = climber;
    this.drivetrain = drivetrain;
    addRequirements(climber);
    this.pidController = new PIDController(autoBalanceKP, autoBalanceKD, autoBalanceKI);
    this.isLongArms = climber.getLongerArms();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    climber.setLeftMotorCurrent(CONSTANT_CLIMBER_CURRENT);
    climber.setRightMotorCurrent(CONSTANT_CLIMBER_CURRENT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(drivetrain.getRoll()) < THRESHOLD){
      climber.setLeftMotorCurrent(CONSTANT_CLIMBER_CURRENT);
      climber.setRightMotorCurrent(CONSTANT_CLIMBER_CURRENT);
    }

    else if (drivetrain.getRoll() > THRESHOLD) {
      climber.setRightMotorCurrent(pidController.calculate(drivetrain.getRoll(), 0));
      climber.setLeftMotorCurrent(0);
    }

    else if (drivetrain.getRoll() < THRESHOLD) {
      climber.setLeftMotorCurrent(pidController.calculate(drivetrain.getRoll(), 0));
      climber.setRightMotorCurrent(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setLeftMotorCurrent(0);
    climber.setRightMotorCurrent(0);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
