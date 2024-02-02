package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
  private VelocityTorqueCurrentFOC drumMotorVelocityRequest;
  private VelocityTorqueCurrentFOC dunkerMotorVelocityRequest;
  // Using motion magic for the Angle Motor

  // Using StatusSignal to get the stator current of the motors
  private StatusSignal<Double> shootMotorLeftStatorCurrentStatusSignal;
  private StatusSignal<Double> shootMotorRightStatorCurrentStatusSignal;
  private StatusSignal<Double> drumMotorStatorCurrentStatusSignal;
  private StatusSignal<Double> dunkerMotorStatorCurrentStatusSignal;
  private StatusSignal<Double> angleMotorStatorCurrentStatusSignal;

  // Using StatusSignal to get the supply current of the motors
  private StatusSignal<Double> shootMotorLeftSupplyCurrentStatusSignal;
  private StatusSignal<Double> shootMotorRightSupplyCurrentStatusSignal;
  private StatusSignal<Double> drumMotorSupplyCurrentStatusSignal;
  private StatusSignal<Double> dunkerMotorSupplyCurrentStatusSignal;
  private StatusSignal<Double> angleMotorSupplyCurrentStatusSignal;

  // Using StatusSignal to get the velocity of the motors
  private StatusSignal<Double> shootMotorLeftVelocityStatusSignal;
  private StatusSignal<Double> shootMotorRightVelocityStatusSignal;
  private StatusSignal<Double> drumMotorVelocityStatusSignal;
  private StatusSignal<Double> dunkerMotorVelocityStatusSignal;

  // Creates the reference velocity request for the motors
  private double shootMotorLeftRequestedVelocityRPS;
  private double shootMotorRightRequestedVelocityRPS;
  private double drumMotorRequestedVelocityRPS;
  private double dunkerMotorVelocityRPS;
  private double angleMotorRequestedVelocityRPS;

  // Shoot PID Tunable Numbers
  private final TunableNumber shootMotorsKP = new TunableNumber("Shooter/SHOOT_KP", ShooterConstants.SHOOT_KP);
  private final TunableNumber shootMotorsKI = new TunableNumber("Shooter/SHOOT_KI", ShooterConstants.SHOOT_KI);
  private final TunableNumber shootMotorsKD = new TunableNumber("Shooter/SHOOT_KD", ShooterConstants.SHOOT_KD);
  private final TunableNumber shootMotorsPeakOutput = new TunableNumber("Shooter/SHOOT_PID_PEAK_OUTPUT", ShooterConstants.SHOOT_PID_PEAK_OUTPUT);

  // Angle PID Tunable Numbers
  private final TunableNumber rotationMotorKP = new TunableNumber("Shooter/ROTATION_KP", ShooterConstants.ROTATION_KP);
  private final TunableNumber rotationMotorKI = new TunableNumber("Shooter/ROTATION_KI", ShooterConstants.ROTATION_KI);
  private final TunableNumber rotationMotorKD = new TunableNumber("Shooter/ROTATION_KD", ShooterConstants.ROTATION_KD);
  private final TunableNumber rotationMotorPeakOutput = new TunableNumber("Shooter/ROTATION_PID_PEAK_OUTPUT", ShooterConstants.ROTATION_PID_PEAK_OUTPUT);

  private TalonFX shootMotorLeft;
  private TalonFX shootMotorRight;
  private TalonFX angleMotor;
  private TalonFX drumMotor;
  private TalonFX dunkerMotor;
  private CANcoder angleEncoder;
  private final DigitalInput shooterSensor;

  public ShooterIOTalonFX() {

    shootMotorLeftVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorRightVelocityRequest = new VelocityTorqueCurrentFOC(0);
    drumMotorVelocityRequest = new VelocityTorqueCurrentFOC(0);
    dunkerMotorVelocityRequest = new VelocityTorqueCurrentFOC(0);

    shootMotorLeftVelocityStatusSignal = shootMotorLeft.getRotorVelocity();
    shootMotorRightVelocityStatusSignal = shootMotorRight.getRotorVelocity();
    drumMotorVelocityStatusSignal = drumMotor.getRotorVelocity();
    dunkerMotorVelocityStatusSignal = dunkerMotor.getRotorVelocity();

    shootMotorLeftStatorCurrentStatusSignal = shootMotorLeft.getStatorCurrent();
    shootMotorRightStatorCurrentStatusSignal = shootMotorRight.getStatorCurrent();
    drumMotorStatorCurrentStatusSignal = drumMotor.getStatorCurrent();
    dunkerMotorStatorCurrentStatusSignal = dunkerMotor.getStatorCurrent();
    angleMotorStatorCurrentStatusSignal = angleMotor.getStatorCurrent();

    shootMotorLeftSupplyCurrentStatusSignal = shootMotorLeft.getSupplyCurrent();
    shootMotorRightSupplyCurrentStatusSignal = shootMotorRight.getSupplyCurrent();
    drumMotorSupplyCurrentStatusSignal = drumMotor.getSupplyCurrent();
    dunkerMotorSupplyCurrentStatusSignal = dunkerMotor.getSupplyCurrent();
    angleMotorSupplyCurrentStatusSignal = angleMotor.getSupplyCurrent();

    configShootMotors(LEFT_SHOOTER_MOTOR_ID, RIGHT_SHOOTER_MOTOR_ID);
    configAngleMotor(ANGLE_MOTOR_ID, ANGLE_ENCODER_ID);
    configDunkerMotor(DUNKER_MOTOR_ID);
    configDrumMotor(DRUM_MOTOR_ID);
    this.shooterSensor = new DigitalInput(ShooterConstants.SHOOTER_SENSOR_ID);
  }

  @Override
  public void updateInputs(ShooterIOInputs shooterInputs){
    BaseStatusSignal.refreshAll(
      shootMotorLeftVelocityStatusSignal,
      shootMotorRightVelocityStatusSignal,
      drumMotorVelocityStatusSignal,
      dunkerMotorVelocityStatusSignal,
      shootMotorLeftStatorCurrentStatusSignal,
      shootMotorRightStatorCurrentStatusSignal,
      drumMotorStatorCurrentStatusSignal,
      dunkerMotorStatorCurrentStatusSignal,
      angleMotorStatorCurrentStatusSignal,
      shootMotorLeftSupplyCurrentStatusSignal,
      shootMotorRightSupplyCurrentStatusSignal,
      drumMotorSupplyCurrentStatusSignal,
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
    shooterInputs.angleMotorVelocityRPS = angleMotor.get();

    // Updates Drum Motor Inputs
    shooterInputs.drumMotorStatorCurrentAmps = drumMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.drumMotorSupplyCurrentAmps = drumMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.drumMotorVelocityRPS = drumMotorVelocityStatusSignal.getValueAsDouble();

    // Updates Dunker Motor Inputs
    shooterInputs.dunkerMotorStatorCurrentAmps = dunkerMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.dunkerMotorSupplyCurrentAmps = dunkerMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.dunkerMotorVelocityRPS = dunkerMotorVelocityStatusSignal.getValueAsDouble();

    // Updates Shooter Sensor
    shooterInputs.isShooterSensorBlocked = shooterSensor.get();    

  }


  @Override
  public void setDrumMotorVelocity(double rps) {
    drumMotor.setControl(drumMotorVelocityRequest.withVelocity(rps));
    this.drumMotorRequestedVelocityRPS = rps;
  }


  @Override
  public void setDunkerMotorVelocity(double rps) {
    dunkerMotor.setControl(dunkerMotorVelocityRequest.withVelocity(rps));
    this.dunkerMotorVelocityRPS = rps;
  }


  @Override
  public void setShooterMotorLeftVelocity(double rps) {
    shootMotorLeft.setControl(shootMotorLeftVelocityRequest.withVelocity(rps));
    this.shootMotorLeftRequestedVelocityRPS = rps;
  }


  @Override
  public void setShooterMotorRightVelocity(double rps) {
    shootMotorRight.setControl(shootMotorRightVelocityRequest.withVelocity(rps));
    this.shootMotorRightRequestedVelocityRPS = rps;
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

      shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      shootMotorsConfig.Slot0.kP = shootMotorsKP.get();
      shootMotorsConfig.Slot0.kI = shootMotorsKI.get();
      shootMotorsConfig.Slot0.kD = shootMotorsKD.get();

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

    angleMotorConfig.MotorOutput.Inverted = ShooterConstants.ANGLE_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;

    angleMotor.getConfigurator().apply(angleMotorConfig);
  }

  private void configDunkerMotor(int dunkerMotorID){
    dunkerMotor = new TalonFX(dunkerMotorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration dunkerMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs dunkerMotorCurrentLimits = new CurrentLimitsConfigs();

    dunkerMotorCurrentLimits.SupplyCurrentLimit = ShooterConstants.DRUM_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    dunkerMotorCurrentLimits.SupplyCurrentThreshold = ShooterConstants.DUNKER_MOTOR_PEAK_CURRENT_LIMIT;
    dunkerMotorCurrentLimits.SupplyTimeThreshold = ShooterConstants.DUNKER_MOTOR_PEAK_CURRENT_DURATION;
    dunkerMotorCurrentLimits.SupplyCurrentLimitEnable = true;

    dunkerMotorConfig.CurrentLimits = dunkerMotorCurrentLimits;

    dunkerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    dunkerMotorConfig.MotorOutput.Inverted = ShooterConstants.DUNKER_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;

    dunkerMotor.getConfigurator().apply(dunkerMotorConfig);
  }

  private void configDrumMotor(int drumMotorID){
    drumMotor = new TalonFX(drumMotorID, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration drumMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs drumMotorCurrentLimits = new CurrentLimitsConfigs();

    drumMotorCurrentLimits.SupplyCurrentLimit = ShooterConstants.DRUM_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    drumMotorCurrentLimits.SupplyCurrentThreshold = ShooterConstants.DRUM_MOTOR_PEAK_CURRENT_LIMIT;
    drumMotorCurrentLimits.SupplyTimeThreshold = ShooterConstants.DRUM_MOTOR_PEAK_CURRENT_DURATION;
    drumMotorCurrentLimits.SupplyCurrentLimitEnable = true;

    drumMotorConfig.CurrentLimits = drumMotorCurrentLimits;

    drumMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    drumMotorConfig.MotorOutput.Inverted = ShooterConstants.DRUM_MOTOR_INVERTED
      ? InvertedValue.Clockwise_Positive
      : InvertedValue.CounterClockwise_Positive;
    
    drumMotor.getConfigurator().apply(drumMotorConfig);
  }
}


