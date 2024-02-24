package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;

public class ShooterIOTalonFX implements ShooterIO {

  // Using VelocityTorqueCurrentFOC to set the velocity of the motors
  private VelocityTorqueCurrentFOC shootMotorTopVelocityRequest;
  private VelocityTorqueCurrentFOC shootMotorBottomVelocityRequest;
  private MotionMagicExpoTorqueCurrentFOC angleMotorPositionRequest;

  // Using StatusSignal to get the stator current of the motors
  private StatusSignal<Double> shootMotorTopStatorCurrentStatusSignal;
  private StatusSignal<Double> shootMotorBottomStatorCurrentStatusSignal;
  private StatusSignal<Double> angleMotorStatorCurrentStatusSignal;

  // Using StatusSignal to get the supply current of the motors
  private StatusSignal<Double> shootMotorTopSupplyCurrentStatusSignal;
  private StatusSignal<Double> shootMotorBottomSupplyCurrentStatusSignal;
  private StatusSignal<Double> angleMotorSupplyCurrentStatusSignal;

  // Using StatusSignal to get the velocity of the motors
  private StatusSignal<Double> shootMotorTopVelocityStatusSignal;
  private StatusSignal<Double> shootMotorBottomVelocityStatusSignal;
  private StatusSignal<Double> angleMotorPositionStatusSignal;

  private StatusSignal<Double> shootMotorTopReferenceVelocityStatusSignal;
  private StatusSignal<Double> shootMotorBottomReferenceVelocityStatusSignal;
  private StatusSignal<Double> angleMotorReferencePositionStatusSignal;

  // Shoot PID Tunable Numbers
  private final TunableNumber shootMotorsKP =
      new TunableNumber("Shooter/SHOOT_KP", ShooterConstants.SHOOT_KP);
  private final TunableNumber shootMotorsKI =
      new TunableNumber("Shooter/SHOOT_KI", ShooterConstants.SHOOT_KI);
  private final TunableNumber shootMotorsKD =
      new TunableNumber("Shooter/SHOOT_KD", ShooterConstants.SHOOT_KD);
  private final TunableNumber shootMotorsKS =
      new TunableNumber("Shooter/SHOOT_KS", ShooterConstants.SHOOT_KS);
  private final TunableNumber shootMotorsPeakOutput =
      new TunableNumber("Shooter/SHOOT_PID_PEAK_OUTPUT", ShooterConstants.SHOOT_PID_PEAK_OUTPUT);

  // Angle PID Tunable Numbers
  private final TunableNumber rotationMotorKP =
      new TunableNumber("Shooter/ROTATION_KP", ShooterConstants.ROTATION_KP);
  private final TunableNumber rotationMotorKI =
      new TunableNumber("Shooter/ROTATION_KI", ShooterConstants.ROTATION_KI);
  private final TunableNumber rotationMotorKD =
      new TunableNumber("Shooter/ROTATION_KD", ShooterConstants.ROTATION_KD);
  private final TunableNumber rotationMotorKS =
      new TunableNumber("Shooter/ROTATION_KS", ShooterConstants.ROTATION_KS);
  private final TunableNumber rotationMotorKG =
      new TunableNumber("Shooter/ROTATION_KG", ShooterConstants.ROTATION_KG);
  private final TunableNumber rotationMotorKA =
      new TunableNumber("Shooter/ROTATION_KA", ShooterConstants.ROTATION_KA);
  private final TunableNumber rotationMotorKV =
      new TunableNumber("Shooter/ROTATION_KV", ShooterConstants.ROTATION_KV);
  private final TunableNumber rotationMotorPeakOutput =
      new TunableNumber(
          "Shooter/ROTATION_PID_PEAK_OUTPUT", ShooterConstants.ROTATION_PID_PEAK_OUTPUT);

  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;
  private TalonFX angleMotor;
  private CANcoder angleEncoder;

  public ShooterIOTalonFX() {

    shootMotorTop = new TalonFX(TOP_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    shootMotorBottom =
        new TalonFX(BOTTOM_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleMotor = new TalonFX(ANGLE_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleEncoder = new CANcoder(ANGLE_ENCODER_ID);

    shootMotorTopVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorBottomVelocityRequest = new VelocityTorqueCurrentFOC(0);
    angleMotorPositionRequest = new MotionMagicExpoTorqueCurrentFOC(0);

    shootMotorTopVelocityStatusSignal = shootMotorTop.getVelocity();
    shootMotorBottomVelocityStatusSignal = shootMotorBottom.getVelocity();
    angleMotorPositionStatusSignal = angleMotor.getPosition();

    shootMotorTopStatorCurrentStatusSignal = shootMotorTop.getStatorCurrent();
    shootMotorBottomStatorCurrentStatusSignal = shootMotorBottom.getStatorCurrent();
    angleMotorStatorCurrentStatusSignal = angleMotor.getStatorCurrent();

    shootMotorTopSupplyCurrentStatusSignal = shootMotorTop.getSupplyCurrent();
    shootMotorBottomSupplyCurrentStatusSignal = shootMotorBottom.getSupplyCurrent();
    angleMotorSupplyCurrentStatusSignal = angleMotor.getSupplyCurrent();

    shootMotorTopReferenceVelocityStatusSignal = shootMotorTop.getClosedLoopReference();
    shootMotorBottomReferenceVelocityStatusSignal = shootMotorBottom.getClosedLoopReference();
    angleMotorReferencePositionStatusSignal = angleMotor.getClosedLoopReference();

    configShootMotor(shootMotorTop, SHOOT_TOP_INVERTED);
    configShootMotor(shootMotorBottom, SHOOT_BOTTOM_INVERTED);
    configAngleMotor(angleMotor, angleEncoder);
  }

  @Override
  public void updateInputs(ShooterIOInputs shooterInputs) {
    BaseStatusSignal.refreshAll(
        shootMotorTopVelocityStatusSignal,
        shootMotorBottomVelocityStatusSignal,
        shootMotorTopStatorCurrentStatusSignal,
        shootMotorBottomStatorCurrentStatusSignal,
        angleMotorStatorCurrentStatusSignal,
        shootMotorTopSupplyCurrentStatusSignal,
        shootMotorBottomSupplyCurrentStatusSignal,
        angleMotorSupplyCurrentStatusSignal);

    // Updates Top Shooter Motor Inputs
    shooterInputs.shootMotorTopStatorCurrentAmps =
        shootMotorTopStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopSupplyCurrentAmps =
        shootMotorTopSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopVelocityRPS = shootMotorTopVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopReferenceVelocityRPS =
        shootMotorTopReferenceVelocityStatusSignal.getValueAsDouble();
    if (shootMotorsKD.hasChanged()
        || shootMotorsKI.hasChanged()
        || shootMotorsKP.hasChanged()
        || shootMotorsKS.hasChanged()) {
      Slot0Configs topShootMotorConfig = new Slot0Configs();
      topShootMotorConfig.kP = shootMotorsKP.get();
      topShootMotorConfig.kI = shootMotorsKI.get();
      topShootMotorConfig.kD = shootMotorsKD.get();
      topShootMotorConfig.kS = shootMotorsKS.get();

      shootMotorTop.getConfigurator().apply(topShootMotorConfig);
    }

    // Updates Bottom Shooter Motor Inputs
    shooterInputs.shootMotorBottomStatorCurrentAmps =
        shootMotorBottomStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomSupplyCurrentAmps =
        shootMotorBottomSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomVelocityRPS =
        shootMotorBottomVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomReferenceVelocityRPS =
        shootMotorBottomReferenceVelocityStatusSignal.getValueAsDouble();
    if (shootMotorsKD.hasChanged()
        || shootMotorsKI.hasChanged()
        || shootMotorsKP.hasChanged()
        || shootMotorsKS.hasChanged()) {
      Slot0Configs bottomShootMotorConfig = new Slot0Configs();
      bottomShootMotorConfig.kP = shootMotorsKP.get();
      bottomShootMotorConfig.kI = shootMotorsKI.get();
      bottomShootMotorConfig.kD = shootMotorsKD.get();
      bottomShootMotorConfig.kS = shootMotorsKS.get();
      shootMotorBottom.getConfigurator().apply(bottomShootMotorConfig);
    }

    // Updates Angle Motor Inputs
    shooterInputs.angleMotorStatorCurrentAmps =
        angleMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorSupplyCurrentAmps =
        angleMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleEncoderAngleDegrees = rotationsToDegrees(angleMotor.getPosition());
    shooterInputs.angleMotorReferenceAngleDegrees =
        angleMotorReferencePositionStatusSignal.getValueAsDouble();
    if (rotationMotorKP.hasChanged()
        || rotationMotorKI.hasChanged()
        || rotationMotorKD.hasChanged()
        || rotationMotorKS.hasChanged()
        || rotationMotorKG.hasChanged()
        || rotationMotorKA.hasChanged()
        || rotationMotorKV.hasChanged()) {
      Slot0Configs angleMotorConfig = new Slot0Configs();
      angleMotorConfig.kP = rotationMotorKP.get();
      angleMotorConfig.kI = rotationMotorKI.get();
      angleMotorConfig.kD = rotationMotorKD.get();
      angleMotorConfig.kS = rotationMotorKS.get();
      angleMotorConfig.kG = rotationMotorKG.get();
      angleMotorConfig.kA = rotationMotorKA.get();
      angleMotorConfig.kV = rotationMotorKV.get();
      angleMotor.getConfigurator().apply(angleMotorConfig);
    }
  }

  @Override
  public void setShooterWheelTopVelocity(double rps) {
    shootMotorTop.setControl(shootMotorTopVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setShooterWheelBottomVelocity(double rps) {
    shootMotorBottom.setControl(shootMotorBottomVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setAngle(double angle) {
    angleMotor.setControl(angleMotorPositionRequest.withPosition(degreesToRotations(angle)));
  }

  private double degreesToRotations(double degrees) {
    return degrees / 360;
  }

  private double rotationsToDegrees(StatusSignal<Double> statusSignal) {
    return statusSignal.getValueAsDouble() * 360;
  }

  private void configShootMotor(TalonFX shootMotor, boolean isInverted) {

    TalonFXConfiguration shootMotorsConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs shootMotorsCurrentLimits = new CurrentLimitsConfigs();

    shootMotorsCurrentLimits.SupplyCurrentLimit =
        ShooterConstants.SHOOT_MOTORS_CONTINUOUS_CURRENT_LIMIT;
    shootMotorsCurrentLimits.SupplyCurrentThreshold =
        ShooterConstants.SHOOT_MOTORS_PEAK_CURRENT_LIMIT;
    shootMotorsCurrentLimits.SupplyTimeThreshold =
        ShooterConstants.SHOOT_MOTORS_PEAK_CURRENT_DURATION;
    shootMotorsCurrentLimits.SupplyCurrentLimitEnable = true;

    shootMotorsConfig.CurrentLimits = shootMotorsCurrentLimits;

    shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shootMotorsConfig.Slot0.kP = shootMotorsKP.get();
    shootMotorsConfig.Slot0.kI = shootMotorsKI.get();
    shootMotorsConfig.Slot0.kD = shootMotorsKD.get();
    shootMotorsConfig.Slot0.kS = shootMotorsKS.get();

    shootMotorsConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    shootMotorsConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    shootMotor.getConfigurator().apply(shootMotorsConfig);
  }

  private void configAngleMotor(TalonFX angleMotor, CANcoder angleEncoder) {

    TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs angleMotorCurrentLimits = new CurrentLimitsConfigs();
    MotionMagicConfigs rotationMotionMagicConfig = angleMotorConfig.MotionMagic;

    angleMotorCurrentLimits.SupplyCurrentLimit =
        ShooterConstants.ANGLE_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    angleMotorCurrentLimits.SupplyCurrentThreshold =
        ShooterConstants.ANGLE_MOTOR_PEAK_CURRENT_LIMIT;
    angleMotorCurrentLimits.SupplyTimeThreshold =
        ShooterConstants.ANGLE_MOTOR_PEAK_CURRENT_DURATION;
    angleMotorCurrentLimits.SupplyCurrentLimitEnable = true;

    angleMotorCurrentLimits.StatorCurrentLimit = ShooterConstants.ANGLE_MOTOR_STATOR_CURRENT_LIMIT;

    angleMotorConfig.CurrentLimits = angleMotorCurrentLimits;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotorConfig.Slot0.kP = rotationMotorKP.get();
    angleMotorConfig.Slot0.kI = rotationMotorKI.get();
    angleMotorConfig.Slot0.kD = rotationMotorKD.get();
    angleMotorConfig.Slot0.kS = rotationMotorKS.get();
    angleMotorConfig.Slot0.kG = rotationMotorKG.get();
    angleMotorConfig.Slot0.kA = rotationMotorKA.get();
    angleMotorConfig.Slot0.kV = rotationMotorKV.get();

    rotationMotionMagicConfig.MotionMagicCruiseVelocity =
        ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    rotationMotionMagicConfig.MotionMagicAcceleration = ShooterConstants.MOTION_MAGIC_ACCELERATION;
    rotationMotionMagicConfig.MotionMagicJerk = ShooterConstants.MOTION_MAGIC_JERK;

    angleMotorConfig.MotorOutput.Inverted =
        ShooterConstants.ANGLE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();

    angleCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    angleCANCoderConfig.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    angleCANCoderConfig.MagnetSensor.MagnetOffset = ShooterConstants.MAGNET_OFFSET;
    angleEncoder.getConfigurator().apply(angleCANCoderConfig);

    angleMotorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SENSOR_TO_MECHANISM_RATIO;
    angleMotorConfig.Feedback.RotorToSensorRatio = ShooterConstants.ROTOR_TO_SENSOR_RATIO;

    angleMotor.getConfigurator().apply(angleMotorConfig);

    angleMotor.setInverted(true);
  }
}