package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;

public class ShooterIOTalonFX implements ShooterIO {

  // Using VelocityTorqueCurrentFOC to set the velocity of the motors
  private VelocityTorqueCurrentFOC shootMotorLeftVelocityRequest;
  private VelocityTorqueCurrentFOC shootMotorRightVelocityRequest;
  private VelocityTorqueCurrentFOC kickerMotorVelocityRequest;
  private VelocityTorqueCurrentFOC dunkerMotorVelocityRequest;
  private MotionMagicExpoTorqueCurrentFOC angleMotorPositionRequest;

  // Using StatusSignal to get the stator current of the motors
  private StatusSignal<Double> shootMotorLeftStatorCurrentStatusSignal;
  private StatusSignal<Double> shootMotorRightStatorCurrentStatusSignal;
  private StatusSignal<Double> kickerMotorStatorCurrentStatusSignal;
  private StatusSignal<Double> dunkerMotorStatorCurrentStatusSignal;
  private StatusSignal<Double> angleMotorStatorCurrentStatusSignal;

  // Using StatusSignal to get the supply current of the motors
  private StatusSignal<Double> shootMotorLeftSupplyCurrentStatusSignal;
  private StatusSignal<Double> shootMotorRightSupplyCurrentStatusSignal;
  private StatusSignal<Double> kickerMotorSupplyCurrentStatusSignal;
  private StatusSignal<Double> dunkerMotorSupplyCurrentStatusSignal;
  private StatusSignal<Double> angleMotorSupplyCurrentStatusSignal;

  // Using StatusSignal to get the velocity of the motors
  private StatusSignal<Double> shootMotorLeftVelocityStatusSignal;
  private StatusSignal<Double> shootMotorRightVelocityStatusSignal;
  private StatusSignal<Double> kickerMotorVelocityStatusSignal;
  private StatusSignal<Double> dunkerMotorVelocityStatusSignal;
  private StatusSignal<Double> angleMotorPositionStatusSignal;

  // Shoot PID Tunable Numbers
  private final TunableNumber shootMotorsKP = new TunableNumber("Shooter/SHOOT_KP", ShooterConstants.SHOOT_KP);
  private final TunableNumber shootMotorsKI = new TunableNumber("Shooter/SHOOT_KI", ShooterConstants.SHOOT_KI);
  private final TunableNumber shootMotorsKD = new TunableNumber("Shooter/SHOOT_KD", ShooterConstants.SHOOT_KD);
  private final TunableNumber shootMotorsKS = new TunableNumber("Shooter/SHOOT_KS", ShooterConstants.SHOOT_KS);
  private final TunableNumber shootMotorsPeakOutput = new TunableNumber("Shooter/SHOOT_PID_PEAK_OUTPUT", ShooterConstants.SHOOT_PID_PEAK_OUTPUT);

  // Angle PID Tunable Numbers
  private final TunableNumber rotationMotorKP = new TunableNumber("Shooter/ROTATION_KP", ShooterConstants.ROTATION_KP);
  private final TunableNumber rotationMotorKI = new TunableNumber("Shooter/ROTATION_KI", ShooterConstants.ROTATION_KI);
  private final TunableNumber rotationMotorKD = new TunableNumber("Shooter/ROTATION_KD", ShooterConstants.ROTATION_KD);
  private final TunableNumber rotationMotorKS = new TunableNumber("Shooter/ROTATION_KS", ShooterConstants.ROTATION_KS);
  private final TunableNumber rotationMotorKG = new TunableNumber("Shooter/ROTATION_KG", ShooterConstants.ROTATION_KG);
  private final TunableNumber rotationMotorKA = new TunableNumber("Shooter/ROTATION_KA", ShooterConstants.ROTATION_KA);
  private final TunableNumber rotationMotorPeakOutput = new TunableNumber("Shooter/ROTATION_PID_PEAK_OUTPUT", ShooterConstants.ROTATION_PID_PEAK_OUTPUT);

  // Dunker PID Tunable Numbers
  private final TunableNumber dunkerMotorKS = new TunableNumber("Shooter/DUNKER_KS", ShooterConstants.DUNKER_KS);

  private TalonFX shootMotorLeft;
  private TalonFX shootMotorRight;
  private TalonFX angleMotor;
  private TalonFX kickerMotor;
  private TalonFX dunkerMotor;
  private CANcoder angleEncoder;
  private final DigitalInput shooterSensor;

  public ShooterIOTalonFX() {

    shootMotorLeftVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorRightVelocityRequest = new VelocityTorqueCurrentFOC(0);
    kickerMotorVelocityRequest = new VelocityTorqueCurrentFOC(0);
    dunkerMotorVelocityRequest = new VelocityTorqueCurrentFOC(0);
    angleMotorPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    shootMotorLeftVelocityStatusSignal = shootMotorLeft.getVelocity();
    shootMotorRightVelocityStatusSignal = shootMotorRight.getVelocity();
    kickerMotorVelocityStatusSignal = kickerMotor.getVelocity();
    dunkerMotorVelocityStatusSignal = dunkerMotor.getVelocity();
    angleMotorPositionStatusSignal= angleMotor.getPosition();

    shootMotorLeftStatorCurrentStatusSignal = shootMotorLeft.getStatorCurrent();
    shootMotorRightStatorCurrentStatusSignal = shootMotorRight.getStatorCurrent();
    kickerMotorStatorCurrentStatusSignal = kickerMotor.getStatorCurrent();
    dunkerMotorStatorCurrentStatusSignal = dunkerMotor.getStatorCurrent();
    angleMotorStatorCurrentStatusSignal = angleMotor.getStatorCurrent();

    shootMotorLeftSupplyCurrentStatusSignal = shootMotorLeft.getSupplyCurrent();
    shootMotorRightSupplyCurrentStatusSignal = shootMotorRight.getSupplyCurrent();
    kickerMotorSupplyCurrentStatusSignal = kickerMotor.getSupplyCurrent();
    dunkerMotorSupplyCurrentStatusSignal = dunkerMotor.getSupplyCurrent();
    angleMotorSupplyCurrentStatusSignal = angleMotor.getSupplyCurrent();

    configShootMotors(LEFT_SHOOTER_MOTOR_ID, RIGHT_SHOOTER_MOTOR_ID);
    configAngleMotor(ANGLE_MOTOR_ID, ANGLE_ENCODER_ID);
    configDunkerMotor(DUNKER_MOTOR_ID);
    configKickerMotor(KICKER_MOTOR_ID);
    this.shooterSensor = new DigitalInput(ShooterConstants.SHOOTER_SENSOR_ID);
  }

  @Override
  public void updateInputs(ShooterIOInputs shooterInputs){
    BaseStatusSignal.refreshAll(
      shootMotorLeftVelocityStatusSignal,
      shootMotorRightVelocityStatusSignal,
      kickerMotorVelocityStatusSignal,
      dunkerMotorVelocityStatusSignal,
      shootMotorLeftStatorCurrentStatusSignal,
      shootMotorRightStatorCurrentStatusSignal,
      kickerMotorStatorCurrentStatusSignal,
      dunkerMotorStatorCurrentStatusSignal,
      angleMotorStatorCurrentStatusSignal,
      shootMotorLeftSupplyCurrentStatusSignal,
      shootMotorRightSupplyCurrentStatusSignal,
      kickerMotorSupplyCurrentStatusSignal,
      dunkerMotorSupplyCurrentStatusSignal,
      angleMotorSupplyCurrentStatusSignal
    );
    
    // Updates Left Shooter Motor Inputs
    shooterInputs.shootMotorLeftStatorCurrentAmps = shootMotorLeftStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorLeftSupplyCurrentAmps = shootMotorLeftSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorLeftVelocityRPS = shootMotorLeftVelocityStatusSignal.getValueAsDouble();

    // Updates Right Shooter Motor Inputs
    shooterInputs.shootMotorRightStatorCurrentAmps = shootMotorRightStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorRightSupplyCurrentAmps = shootMotorRightSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorRightVelocityRPS = shootMotorRightVelocityStatusSignal.getValueAsDouble();

    // Updates Angle Motor Inputs
    shooterInputs.angleMotorStatorCurrentAmps = angleMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorSupplyCurrentAmps = angleMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleEncoderAngleDegrees = angleMotor.get();

    // Updates Kicker Motor Inputs
    shooterInputs.kickerMotorStatorCurrentAmps = kickerMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.kickerMotorSupplyCurrentAmps = kickerMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.kickerMotorVelocityRPS = kickerMotorVelocityStatusSignal.getValueAsDouble();

    // Updates Dunker Motor Inputs
    shooterInputs.dunkerMotorStatorCurrentAmps = dunkerMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.dunkerMotorSupplyCurrentAmps = dunkerMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.dunkerMotorVelocityRPS = dunkerMotorVelocityStatusSignal.getValueAsDouble();

    // Updates Shooter Sensor
    shooterInputs.isKickerSensorBlocked = shooterSensor.get();    

  }


  @Override
  public void setKickerWheelVelocity(double rps) {
    kickerMotor.setControl(kickerMotorVelocityRequest.withVelocity(rps));
  }


  @Override
  public void setDunkerMotorVelocity(double rps) {
    dunkerMotor.setControl(dunkerMotorVelocityRequest.withVelocity(rps));
  }


  @Override
  public void setShooterWheelLeftVelocity(double rps) {
    shootMotorLeft.setControl(shootMotorLeftVelocityRequest.withVelocity(rps));
  }


  @Override
  public void setShooterWheelRightVelocity(double rps) {
    shootMotorRight.setControl(shootMotorRightVelocityRequest.withVelocity(rps));
  }


  @Override
  public void setAngle(double angle) {
    // Magic Motion
  }  


  private void configShootMotors(int leftMotorID, int rightMotorID){
      shootMotorLeft = new TalonFX(leftMotorID, RobotConfig.getInstance().getCANBusName());
      shootMotorRight = new TalonFX(rightMotorID,RobotConfig.getInstance().getCANBusName());

      TalonFXConfiguration shootMotorsConfig = new TalonFXConfiguration();
      CurrentLimitsConfigs shootMotorsCurrentLimits = new CurrentLimitsConfigs();

      shootMotorsCurrentLimits.SupplyCurrentLimit = ShooterConstants.SHOOT_MOTORS_CONTINUOUS_CURRENT_LIMIT;
      shootMotorsCurrentLimits.SupplyCurrentThreshold = ShooterConstants.SHOOT_MOTORS_PEAK_CURRENT_LIMIT;
      shootMotorsCurrentLimits.SupplyTimeThreshold = ShooterConstants.SHOOT_MOTORS_PEAK_CURRENT_DURATION;
      shootMotorsCurrentLimits.SupplyCurrentLimitEnable = true;

      shootMotorsConfig.CurrentLimits = shootMotorsCurrentLimits;

      shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
      shootMotorsConfig.Slot0.kP = shootMotorsKP.get();
      shootMotorsConfig.Slot0.kI = shootMotorsKI.get();
      shootMotorsConfig.Slot0.kD = shootMotorsKD.get();
      shootMotorsConfig.Slot0.kS = shootMotorsKS.get();

      shootMotorsConfig.MotorOutput.Inverted =
        ShooterConstants.SHOOT_LEFT_INVERTED // TODO: What Should I do for the Right Shoot Motor?
          ? InvertedValue.Clockwise_Positive
          : InvertedValue.CounterClockwise_Positive;

      shootMotorLeft.getConfigurator().apply(shootMotorsConfig);
      shootMotorRight.getConfigurator().apply(shootMotorsConfig);
  }

  private void configAngleMotor(int angleMotorID, int angleEncoderID){
    angleMotor = new TalonFX(angleMotorID, RobotConfig.getInstance().getCANBusName());
    angleEncoder = new CANcoder(angleEncoderID);

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs angleMotorCurrentLimits = new CurrentLimitsConfigs();    
    
    // TODO: how to do want the CAN Configuration to be done?


    angleMotorCurrentLimits.SupplyCurrentLimit = ShooterConstants.ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    angleMotorCurrentLimits.SupplyCurrentThreshold = ShooterConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorCurrentLimits.SupplyTimeThreshold = ShooterConstants.ANGLE_MOTOR_PEAK_CURRENT_DURATION;
    angleMotorCurrentLimits.SupplyCurrentLimitEnable = true;

    angleMotorConfig.CurrentLimits = angleMotorCurrentLimits;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotorConfig.Slot0.kP = rotationMotorKP.get();
    angleMotorConfig.Slot0.kI = rotationMotorKI.get();
    angleMotorConfig.Slot0.kD = rotationMotorKD.get();
    angleMotorConfig.Slot0.kS = rotationMotorKS.get();
    angleMotorConfig.Slot0.kG = rotationMotorKG.get();
    angleMotorConfig.Slot0.kA = rotationMotorKA.get();

    angleMotorConfig.MotorOutput.Inverted = ShooterConstants.ANGLE_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;

    angleMotor.getConfigurator().apply(angleMotorConfig);
  }

  private void configDunkerMotor(int dunkerMotorID){
    dunkerMotor = new TalonFX(dunkerMotorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration dunkerMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs dunkerMotorCurrentLimits = new CurrentLimitsConfigs();

    dunkerMotorCurrentLimits.SupplyCurrentLimit = ShooterConstants.DUNKER_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    dunkerMotorCurrentLimits.SupplyCurrentThreshold = ShooterConstants.DUNKER_MOTOR_PEAK_CURRENT_LIMIT;
    dunkerMotorCurrentLimits.SupplyTimeThreshold = ShooterConstants.DUNKER_MOTOR_PEAK_CURRENT_DURATION;
    dunkerMotorCurrentLimits.SupplyCurrentLimitEnable = true;

    dunkerMotorConfig.CurrentLimits = dunkerMotorCurrentLimits;

    dunkerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    dunkerMotorConfig.Slot0.kS = dunkerMotorKS.get();

    dunkerMotorConfig.MotorOutput.Inverted = ShooterConstants.DUNKER_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;

    dunkerMotor.getConfigurator().apply(dunkerMotorConfig);
  }

  private void configKickerMotor(int kickerMotorID){
    kickerMotor = new TalonFX(kickerMotorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration kickerMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs kickerMotorCurrentLimits = new CurrentLimitsConfigs();

    kickerMotorCurrentLimits.SupplyCurrentLimit = ShooterConstants.KICKER_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    kickerMotorCurrentLimits.SupplyCurrentThreshold = ShooterConstants.KICKER_MOTOR_PEAK_CURRENT_LIMIT;
    kickerMotorCurrentLimits.SupplyTimeThreshold = ShooterConstants.KICKER_MOTOR_PEAK_CURRENT_DURATION;
    kickerMotorCurrentLimits.SupplyCurrentLimitEnable = true;

    kickerMotorConfig.CurrentLimits = kickerMotorCurrentLimits;

    kickerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    kickerMotorConfig.MotorOutput.Inverted = ShooterConstants.KICKER_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    
    kickerMotor.getConfigurator().apply(kickerMotorConfig);
  }
}


