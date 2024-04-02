// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Initially from https://github.com/Mechanical-Advantage/RobotCode2022
 */

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj2.command.button.*;

/** Interface for all driver and operator controls. */
public interface OperatorInterface {

  // drivetrain, generic

  public default double getTranslateX() {
    return 0.0;
  }

  public default double getTranslateY() {
    return 0.0;
  }

  public default double getRotate() {
    return 0.0;
  }

  public default Trigger getFieldRelativeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetGyroButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getResetPoseToVisionButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getXStanceButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getTranslationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getRotationSlowModeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getLock180Button() {
    return new Trigger(() -> false);
  }

  public default Trigger getVisionIsEnabledSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getTurboButton() {
    return new Trigger(() -> false);
  }

  // drivetrain, game-specific

  public default Trigger getAimSpeakerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAimAndShootSpeakerButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAlignAndIntakeNoteFromSourceButton() {
    return new Trigger(() -> false);
  }

  // intake

  public default Trigger getIntakeAutomationSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getRunIntakeButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getOuttakeAllButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getIntakeIRSensorSwitch() {
    return new Trigger(() -> false);
  }

  // shooter

  public default Trigger getShootButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShootFullFieldButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPrepareToScoreAmpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPrepareToScoreSubwooferButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getPrepareToScorePodiumButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getShooterAngleDownButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getAimAutomationSwitch() {
    return new Trigger(() -> false);
  }

  public default Trigger getShooterAngleUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getStoreShooterButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getScaleDownShooterVelocityButton() {
    return new Trigger(() -> false);
  }

  // climber

  public default Trigger getClimberDeployButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getFinishClimbButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getEnableManualClimberButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getDisableManualClimberButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getClimberUpButton() {
    return new Trigger(() -> false);
  }

  public default Trigger getClimberDownButton() {
    return new Trigger(() -> false);
  }

  // miscellaneous
  public default Trigger getInterruptAll() {
    return new Trigger(() -> false);
  }
}
