// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.operator_interface;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Class for controlling the robot with two joysticks, 1 Xbox controller, and 1 operator button
 * panel.
 */
public class FullOperatorConsoleOI implements OperatorInterface {
  private final CommandJoystick translateJoystick;
  private final Trigger[] translateJoystickButtons;

  private final CommandJoystick rotateJoystick;
  private final Trigger[] rotateJoystickButtons;

  private final XboxController operatorController;

  private final CommandJoystick operatorPanel;
  private final Trigger[] operatorPanelButtons;

  public FullOperatorConsoleOI(
      int translatePort, int rotatePort, int operatorControllerPort, int operatorPanelPort) {
    translateJoystick = new CommandJoystick(translatePort);
    rotateJoystick = new CommandJoystick(rotatePort);
    operatorController = new XboxController(operatorControllerPort);
    operatorPanel = new CommandJoystick(operatorPanelPort);

    // buttons use 1-based indexing such that the index matches the button number; leave index 0 set
    // to null
    this.translateJoystickButtons = new Trigger[13];
    this.rotateJoystickButtons = new Trigger[13];
    this.operatorPanelButtons = new Trigger[13];

    for (int i = 1; i < translateJoystickButtons.length; i++) {
      translateJoystickButtons[i] = translateJoystick.button(i);
      rotateJoystickButtons[i] = rotateJoystick.button(i);
    }
    for (int i = 1; i < operatorPanelButtons.length; i++) {
      operatorPanelButtons[i] = operatorPanel.button(i);
    }
  }

  // Translate Joystick
  @Override
  public double getTranslateX() {
    return -translateJoystick.getY();
  }

  @Override
  public double getTranslateY() {
    return -translateJoystick.getX();
  }

  @Override
  public Trigger getAimSpeakerButton() {
    return translateJoystickButtons[1];
  }

  @Override
  public Trigger getShootFullFieldButton() {
    return translateJoystickButtons[2];
  }

  @Override
  public Trigger getLock180Button() {
    return translateJoystickButtons[3];
  }

  @Override
  public Trigger getAlignAndIntakeNoteFromSourceButton() {
    return translateJoystickButtons[4];
  }

  @Override
  public Trigger getXStanceButton() {
    return translateJoystickButtons[5];
  }

  @Override
  public Trigger getResetGyroButton() {
    return translateJoystickButtons[8];
  }

  @Override
  public Trigger getFieldRelativeButton() {
    return translateJoystickButtons[9];
  }

  // Rotate Joystick

  @Override
  public double getRotate() {
    return -rotateJoystick.getX();
  }

  @Override
  public Trigger getShootButton() {
    return rotateJoystickButtons[1];
  }

  @Override
  public Trigger getOuttakeAllButton() {
    return rotateJoystickButtons[2];
  }

  @Override
  public Trigger getPrepareToScoreAmpButton() {
    return rotateJoystickButtons[3];
  }

  @Override
  public Trigger getRunIntakeButton() {
    return rotateJoystickButtons[4];
  }

  @Override
  public Trigger getResetPoseToVisionButton() {
    return rotateJoystickButtons[5];
  }

  // Operator Controller

  @Override
  public Trigger getClimberUpButton() {
    return new Trigger(operatorController::getLeftBumper);
  }

  @Override
  public Trigger getClimberDownButton() {
    return new Trigger(operatorController::getRightBumper);
  }

  @Override
  public Trigger getShooterAngleDownButton() {
    return new Trigger(operatorController::getAButton);
  }

  @Override
  public Trigger getShooterAngleUpButton() {
    return new Trigger(operatorController::getYButton);
  }

  @Override
  public Trigger getInterruptAll() {
    return new Trigger(operatorController::getStartButton);
  }

  @Override
  public Trigger getEnableManualClimberButton() {
    return new Trigger(() -> operatorController.getPOV() == 270);
  }

  @Override
  public Trigger getDisableManualClimberButton() {
    return new Trigger(() -> operatorController.getPOV() == 90);
  }

  // Operator Panel

  @Override
  public Trigger getPrepareToScoreSubwooferButton() {
    return operatorPanelButtons[6];
  }

  @Override
  public Trigger getFinishClimbButton() {
    return operatorPanelButtons[7];
  }

  @Override
  public Trigger getPrepareToScorePodiumButton() {
    return operatorPanelButtons[5];
  }

  @Override
  public Trigger getClimberDeployButton() {
    return operatorPanelButtons[8];
  }

  @Override
  public Trigger getVisionIsEnabledSwitch() {
    return operatorPanelButtons[10];
  }

  @Override
  public Trigger getAimAutomationSwitch() {
    return operatorPanelButtons[11];
  }

  @Override
  public Trigger getStoreShooterButton() {
    return operatorPanelButtons[2];
  }

  @Override
  public Trigger getIntakeAutomationSwitch() {
    return operatorPanelButtons[12];
  }

  @Override
  public Trigger getScaleDownShooterVelocityButton() {
    return operatorPanelButtons[9];
  }
}
