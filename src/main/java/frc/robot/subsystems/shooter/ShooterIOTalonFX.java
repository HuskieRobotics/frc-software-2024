package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.ArmSystemSim;
import frc.lib.team3061.util.VelocitySystemSim;
import frc.lib.team6328.util.TunableNumber;

public class ShooterIOTalonFX implements ShooterIO {

  // Using VelocityTorqueCurrentFOC to set the velocity of the motors
  private VelocityTorqueCurrentFOC shootMotorTopVelocityRequest;
  private VelocityTorqueCurrentFOC shootMotorBottomVelocityRequest;
  private MotionMagicExpoVoltage angleMotorPositionRequest;

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

  private StatusSignal<Double> shootMotorTopTemperatureStatusSignal;
  private StatusSignal<Double> shootMotorBottomTemperatureStatusSignal;
  private StatusSignal<Double> angleMotorTemperatureStatusSignal;

  private StatusSignal<Double> shootMotorTopVoltageStatusSignal;
  private StatusSignal<Double> shootMotorBottomVoltageStatusSignal;
  private StatusSignal<Double> angleMotorVoltageStatusSignal;

  private StatusSignal<Double> angleMotorClosedLoopReferenceSlopeStatusSignal;

  private VelocitySystemSim shootMotorTopSim;
  private VelocitySystemSim shootMotorBottomSim;
  private ArmSystemSim angleMotorSim;

  // Shoot PID Tunable Numbers
  private final TunableNumber shootMotorsKP =
      new TunableNumber("Shooter/SHOOT_KP", ShooterConstants.SHOOT_KP);
  private final TunableNumber shootMotorsKI =
      new TunableNumber("Shooter/SHOOT_KI", ShooterConstants.SHOOT_KI);
  private final TunableNumber shootMotorsKD =
      new TunableNumber("Shooter/SHOOT_KD", ShooterConstants.SHOOT_KD);
  private final TunableNumber shootMotorsKS =
      new TunableNumber("Shooter/SHOOT_KS", ShooterConstants.SHOOT_KS);

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
  private final TunableNumber rotationMotorExpoKV =
      new TunableNumber("Shooter/ROTATION_EXPO_KV", ShooterConstants.ROTATION_EXPO_KV);
  private final TunableNumber rotationMotorExpoKA =
      new TunableNumber("Shooter/ROTATION_EXPO_KA", ShooterConstants.ROTATION_EXPO_KA);

  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;
  private TalonFX angleMotor;
  private CANcoder angleEncoder;

  public ShooterIOTalonFX() {

    shootMotorTop = new TalonFX(TOP_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    shootMotorBottom =
        new TalonFX(BOTTOM_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleMotor = new TalonFX(ANGLE_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleEncoder = new CANcoder(ANGLE_ENCODER_ID, RobotConfig.getInstance().getCANBusName());

    shootMotorTopVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorBottomVelocityRequest = new VelocityTorqueCurrentFOC(0);
    angleMotorPositionRequest = new MotionMagicExpoVoltage(0);

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

    shootMotorTopTemperatureStatusSignal = shootMotorTop.getDeviceTemp();
    shootMotorBottomTemperatureStatusSignal = shootMotorBottom.getDeviceTemp();
    angleMotorTemperatureStatusSignal = angleMotor.getDeviceTemp();

    shootMotorTopVoltageStatusSignal = shootMotorTop.getMotorVoltage();
    shootMotorBottomVoltageStatusSignal = shootMotorBottom.getMotorVoltage();
    angleMotorVoltageStatusSignal = angleMotor.getMotorVoltage();

    angleMotorClosedLoopReferenceSlopeStatusSignal = angleMotor.getClosedLoopReferenceSlope();

    configShootMotor(shootMotorTop, SHOOT_TOP_INVERTED);
    configShootMotor(shootMotorBottom, SHOOT_BOTTOM_INVERTED);
    configAngleMotor(angleMotor, angleEncoder);

    this.shootMotorBottomSim =
        new VelocitySystemSim(
            shootMotorBottom,
            ShooterConstants.SHOOT_BOTTOM_INVERTED,
            0.05,
            0.01,
            ShooterConstants.SHOOT_MOTORS_GEAR_RATIO);
    this.shootMotorTopSim =
        new VelocitySystemSim(
            shootMotorTop,
            ShooterConstants.SHOOT_TOP_INVERTED,
            0.05,
            0.01,
            ShooterConstants.SHOOT_MOTORS_GEAR_RATIO);
    this.angleMotorSim =
        new ArmSystemSim(
            angleMotor,
            angleEncoder,
            ShooterConstants.ANGLE_MOTOR_INVERTED,
            ShooterConstants.SENSOR_TO_MECHANISM_RATIO,
            ShooterConstants.ANGLE_MOTOR_GEAR_RATIO,
            Units.inchesToMeters(20.0),
            Units.lbsToKilograms(20.0),
            Units.degreesToRadians(10.0),
            Units.degreesToRadians(100.0),
            Units.degreesToRadians(10.0),
            SUBSYSTEM_NAME);
  }

  @Override
  public void updateInputs(ShooterIOInputs shooterInputs) {
    this.shootMotorBottomSim.updateSim();
    this.shootMotorTopSim.updateSim();
    this.angleMotorSim.updateSim();

    BaseStatusSignal.refreshAll(
        shootMotorTopVelocityStatusSignal,
        shootMotorBottomVelocityStatusSignal,
        shootMotorTopStatorCurrentStatusSignal,
        shootMotorBottomStatorCurrentStatusSignal,
        angleMotorStatorCurrentStatusSignal,
        shootMotorTopSupplyCurrentStatusSignal,
        shootMotorBottomSupplyCurrentStatusSignal,
        angleMotorSupplyCurrentStatusSignal,
        shootMotorTopReferenceVelocityStatusSignal,
        shootMotorBottomReferenceVelocityStatusSignal,
        angleMotorReferencePositionStatusSignal,
        shootMotorTopTemperatureStatusSignal,
        shootMotorBottomTemperatureStatusSignal,
        angleMotorTemperatureStatusSignal,
        shootMotorTopVoltageStatusSignal,
        shootMotorBottomVoltageStatusSignal,
        angleMotorVoltageStatusSignal,
        angleMotorClosedLoopReferenceSlopeStatusSignal);

    // Updates Top Shooter Motor Inputs
    shooterInputs.shootMotorTopStatorCurrentAmps =
        shootMotorTopStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopSupplyCurrentAmps =
        shootMotorTopSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopVelocityRPS = shootMotorTopVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopReferenceVelocityRPS =
        shootMotorTopReferenceVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopTemperatureCelsius =
        shootMotorTopTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopVoltage = shootMotorTopVoltageStatusSignal.getValueAsDouble();

    // Updates Bottom Shooter Motor Inputs
    shooterInputs.shootMotorBottomStatorCurrentAmps =
        shootMotorBottomStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomSupplyCurrentAmps =
        shootMotorBottomSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomVelocityRPS =
        shootMotorBottomVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomReferenceVelocityRPS =
        shootMotorBottomReferenceVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomTemperatureCelsius =
        shootMotorBottomTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomVoltage = shootMotorBottomVoltageStatusSignal.getValueAsDouble();

    // check if the tunable numbers have changed for the top and bottom shooter motors
    if (shootMotorsKD.hasChanged()
        || shootMotorsKI.hasChanged()
        || shootMotorsKP.hasChanged()
        || shootMotorsKS.hasChanged()) {
      Slot0Configs shootMotorConfig = new Slot0Configs();
      shootMotorConfig.kP = shootMotorsKP.get();
      shootMotorConfig.kI = shootMotorsKI.get();
      shootMotorConfig.kD = shootMotorsKD.get();
      shootMotorConfig.kS = shootMotorsKS.get();
      shootMotorTop.getConfigurator().apply(shootMotorConfig);
      shootMotorBottom.getConfigurator().apply(shootMotorConfig);
    }

    // Updates Angle Motor Inputs
    shooterInputs.angleMotorStatorCurrentAmps =
        angleMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorSupplyCurrentAmps =
        angleMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorVoltage = angleMotor.getMotorVoltage().getValue();
    shooterInputs.angleEncoderAngleDegrees =
        Units.rotationsToDegrees(angleMotor.getPosition().getValueAsDouble());
    shooterInputs.angleMotorReferenceAngleDegrees =
        rotationsToDegrees(angleMotorReferencePositionStatusSignal.getValueAsDouble());
    shooterInputs.angleMotorTemperatureCelsius =
        angleMotorTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorClosedLoopReferenceSlope =
        angleMotorClosedLoopReferenceSlopeStatusSignal.getValueAsDouble();
    if (rotationMotorKP.hasChanged()
        || rotationMotorKI.hasChanged()
        || rotationMotorKD.hasChanged()
        || rotationMotorKS.hasChanged()
        || rotationMotorKG.hasChanged()
        || rotationMotorKA.hasChanged()
        || rotationMotorKV.hasChanged()
        || rotationMotorExpoKV.hasChanged()
        || rotationMotorExpoKA.hasChanged()) {
      Slot0Configs angleMotorConfig = new Slot0Configs();
      angleMotorConfig.kP = rotationMotorKP.get();
      angleMotorConfig.kI = rotationMotorKI.get();
      angleMotorConfig.kD = rotationMotorKD.get();
      angleMotorConfig.kS = rotationMotorKS.get();
      angleMotorConfig.kG = rotationMotorKG.get();
      angleMotorConfig.kA = rotationMotorKA.get();
      angleMotorConfig.kV = rotationMotorKV.get();
      angleMotor.getConfigurator().apply(angleMotorConfig);

      MotionMagicConfigs rotationMotionMagicConfig = new MotionMagicConfigs();
      rotationMotionMagicConfig.MotionMagicExpo_kV = rotationMotorExpoKV.get();
      rotationMotionMagicConfig.MotionMagicExpo_kA = rotationMotorExpoKA.get();
      rotationMotionMagicConfig.MotionMagicCruiseVelocity =
          ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
      angleMotor.getConfigurator().apply(rotationMotionMagicConfig);
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
    angleMotor.setControl(angleMotorPositionRequest.withPosition(Units.degreesToRotations(angle)));
  }

  private double rotationsToDegrees(double rotations) {
    return rotations * 360;
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
    angleMotorConfig.CurrentLimits = angleMotorCurrentLimits;

    angleMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleMotorConfig.Slot0.kP = rotationMotorKP.get();
    angleMotorConfig.Slot0.kI = rotationMotorKI.get();
    angleMotorConfig.Slot0.kD = rotationMotorKD.get();
    angleMotorConfig.Slot0.kS = rotationMotorKS.get();
    angleMotorConfig.Slot0.kG = rotationMotorKG.get();
    angleMotorConfig.Slot0.withGravityType(GravityTypeValue.Arm_Cosine);
    angleMotorConfig.Slot0.kA = rotationMotorKA.get();
    angleMotorConfig.Slot0.kV = rotationMotorKV.get();

    rotationMotionMagicConfig.MotionMagicCruiseVelocity =
        ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
    rotationMotionMagicConfig.MotionMagicExpo_kV = rotationMotorExpoKV.get();
    rotationMotionMagicConfig.MotionMagicExpo_kA = rotationMotorExpoKA.get();

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
    angleMotorConfig.Feedback.RotorToSensorRatio = ShooterConstants.ANGLE_MOTOR_GEAR_RATIO;

    angleMotor.getConfigurator().apply(angleMotorConfig);
  }
}
