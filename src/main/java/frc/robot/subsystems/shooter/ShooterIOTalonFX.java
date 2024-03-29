package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.ArmSystemSim;
import frc.lib.team3061.util.VelocitySystemSim;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

public class ShooterIOTalonFX implements ShooterIO {

  // Using VelocityTorqueCurrentFOC to set the velocity of the motors
  private VelocityTorqueCurrentFOC shootMotorTopVelocityRequest;
  private VelocityTorqueCurrentFOC shootMotorBottomVelocityRequest;
  private MotionMagicExpoVoltage angleMotorPositionRequest;
  private VoltageOut angleMotorVoltageRequest;
  private VoltageOut deflectorMotorVoltageRequest;

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

  private StatusSignal<Double> deflectorMotorVoltageStatusSignal;
  private StatusSignal<Double> deflectorMotorStatorCurrentStatusSignal;
  private StatusSignal<Double> deflectorMotorSupplyCurrentStatusSignal;
  private StatusSignal<Double> deflectorMotorTemperatureStatusSignal;

  private StatusSignal<Double> angleMotorClosedLoopReferenceSlopeStatusSignal;

  private VelocitySystemSim shootMotorTopSim;
  private VelocitySystemSim shootMotorBottomSim;
  private ArmSystemSim angleMotorSim;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  // Shoot PID Tunable Numbers
  private final TunableNumber shootMotorTopKP =
      new TunableNumber("Shooter/SHOOT_TOP_KP", ShooterConstants.TOP_SHOOT_KP);
  private final TunableNumber shootMotorTopKI =
      new TunableNumber("Shooter/SHOOT_TOP_KI", ShooterConstants.TOP_SHOOT_KI);
  private final TunableNumber shootMotorTopKD =
      new TunableNumber("Shooter/SHOOT_TOP_KD", ShooterConstants.TOP_SHOOT_KD);
  private final TunableNumber shootMotorTopKS =
      new TunableNumber("Shooter/SHOOT_TOP_KS", ShooterConstants.TOP_SHOOT_KS);

  private final TunableNumber shootMotorBottomKP =
      new TunableNumber("Shooter/SHOOT_BOTTOM_KP", ShooterConstants.BOTTOM_SHOOT_KP);
  private final TunableNumber shootMotorBottomKI =
      new TunableNumber("Shooter/SHOOT__BOTTOM_KI", ShooterConstants.BOTTOM_SHOOT_KI);
  private final TunableNumber shootMotorBottomKD =
      new TunableNumber("Shooter/SHOOT_BOTTOM_KD", ShooterConstants.BOTTOM_SHOOT_KD);
  private final TunableNumber shootMotorBottomKS =
      new TunableNumber("Shooter/SHOOT_BOTTOM_KS", ShooterConstants.BOTTOM_SHOOT_KS);

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

  private final TunableNumber rotationEncoderMagnetOffset =
      new TunableNumber("Shooter/ROTATION_MAGNET_OFFSET", ShooterConstants.MAGNET_OFFSET);

  private TalonFX shootMotorTop;
  private TalonFX shootMotorBottom;
  private TalonFX angleMotor;
  private TalonFX deflectorMotor;
  private CANcoder angleEncoder;

  private DigitalInput coastModeButton;

  private double angleSetpoint;
  private double topWheelVelocity;
  private double bottomWheelVelocity;

  public ShooterIOTalonFX() {

    shootMotorTop = new TalonFX(TOP_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    shootMotorBottom =
        new TalonFX(BOTTOM_SHOOTER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleMotor = new TalonFX(ANGLE_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    angleEncoder = new CANcoder(ANGLE_ENCODER_ID, RobotConfig.getInstance().getCANBusName());

    deflectorMotor = new TalonFX(DEFLECTOR_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    coastModeButton = new DigitalInput(COAST_BUTTON_ID);

    shootMotorTopVelocityRequest = new VelocityTorqueCurrentFOC(0);
    shootMotorBottomVelocityRequest = new VelocityTorqueCurrentFOC(0);
    angleMotorPositionRequest = new MotionMagicExpoVoltage(0);
    angleMotorVoltageRequest = new VoltageOut(0);

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

    deflectorMotorVoltageStatusSignal = deflectorMotor.getMotorVoltage();
    deflectorMotorStatorCurrentStatusSignal = deflectorMotor.getStatorCurrent();
    deflectorMotorSupplyCurrentStatusSignal = deflectorMotor.getSupplyCurrent();
    deflectorMotorTemperatureStatusSignal = deflectorMotor.getDeviceTemp();

    deflectorMotorVoltageRequest = new VoltageOut(0);

    configShootMotor(shootMotorTop, SHOOT_TOP_INVERTED, true);
    configShootMotor(shootMotorBottom, SHOOT_BOTTOM_INVERTED, false);
    configAngleMotor(angleMotor, angleEncoder);
    configDeflectorMotor();

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
            Units.degreesToRadians(120.0),
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
        angleMotorPositionStatusSignal,
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

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.

    // Updates Top Shooter Motor Inputs
    shooterInputs.shootMotorTopStatorCurrentAmps =
        shootMotorTopStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopSupplyCurrentAmps =
        shootMotorTopSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopVelocityRPS = shootMotorTopVelocityStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorTopReferenceVelocityRPS = this.topWheelVelocity;
    shooterInputs.shootMotorTopClosedLoopReferenceRPS =
        shootMotorTop.getClosedLoopReference().getValueAsDouble();
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
    shooterInputs.shootMotorBottomReferenceVelocityRPS = this.bottomWheelVelocity;
    shooterInputs.shootMotorBottomClosedLoopReferenceRPS =
        shootMotorBottom.getClosedLoopReference().getValueAsDouble();
    shooterInputs.shootMotorBottomTemperatureCelsius =
        shootMotorBottomTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.shootMotorBottomVoltage = shootMotorBottomVoltageStatusSignal.getValueAsDouble();

    // check if the tunable numbers have changed for the top and bottom shooter motors
    if (shootMotorTopKD.hasChanged()
        || shootMotorTopKI.hasChanged()
        || shootMotorTopKP.hasChanged()
        || shootMotorTopKS.hasChanged()) {
      Slot0Configs shootMotorConfig = new Slot0Configs();
      shootMotorTop.getConfigurator().refresh(shootMotorConfig);
      shootMotorConfig.kP = shootMotorTopKP.get();
      shootMotorConfig.kI = shootMotorTopKI.get();
      shootMotorConfig.kD = shootMotorTopKD.get();
      shootMotorConfig.kS = shootMotorTopKS.get();

      shootMotorTop.getConfigurator().apply(shootMotorConfig);
    }

    if (shootMotorBottomKD.hasChanged()
        || shootMotorBottomKI.hasChanged()
        || shootMotorBottomKP.hasChanged()
        || shootMotorBottomKS.hasChanged()) {
      Slot0Configs shootMotorConfig = new Slot0Configs();
      shootMotorBottom.getConfigurator().refresh(shootMotorConfig);
      shootMotorConfig.kP = shootMotorBottomKP.get();
      shootMotorConfig.kI = shootMotorBottomKI.get();
      shootMotorConfig.kD = shootMotorBottomKD.get();
      shootMotorConfig.kS = shootMotorBottomKS.get();

      shootMotorBottom.getConfigurator().apply(shootMotorConfig);
    }

    // Updates Angle Motor Inputs
    shooterInputs.angleMotorStatorCurrentAmps =
        angleMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorSupplyCurrentAmps =
        angleMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorVoltage = angleMotor.getMotorVoltage().getValue();
    shooterInputs.angleEncoderAngleDegrees =
        Units.rotationsToDegrees(angleMotorPositionStatusSignal.getValueAsDouble());
    shooterInputs.angleMotorReferenceAngleDegrees = this.angleSetpoint;
    shooterInputs.angleMotorClosedLoopReferenceDegrees =
        Units.rotationsToDegrees(angleMotor.getClosedLoopReference().getValueAsDouble());
    shooterInputs.angleMotorTemperatureCelsius =
        angleMotorTemperatureStatusSignal.getValueAsDouble();
    shooterInputs.angleMotorClosedLoopReferenceSlope =
        angleMotor.getClosedLoopReferenceSlope().getValueAsDouble();
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
      angleMotor.getConfigurator().refresh(angleMotorConfig);
      angleMotorConfig.kP = rotationMotorKP.get();
      angleMotorConfig.kI = rotationMotorKI.get();
      angleMotorConfig.kD = rotationMotorKD.get();
      angleMotorConfig.kS = rotationMotorKS.get();
      angleMotorConfig.kG = rotationMotorKG.get();
      angleMotorConfig.kA = rotationMotorKA.get();
      angleMotorConfig.kV = rotationMotorKV.get();
      angleMotor.getConfigurator().apply(angleMotorConfig);

      MotionMagicConfigs rotationMotionMagicConfig = new MotionMagicConfigs();
      angleMotor.getConfigurator().refresh(rotationMotionMagicConfig);
      rotationMotionMagicConfig.MotionMagicExpo_kV = rotationMotorExpoKV.get();
      rotationMotionMagicConfig.MotionMagicExpo_kA = rotationMotorExpoKA.get();
      rotationMotionMagicConfig.MotionMagicCruiseVelocity =
          ShooterConstants.MOTION_MAGIC_CRUISE_VELOCITY;
      angleMotor.getConfigurator().apply(rotationMotionMagicConfig);
    }

    if (rotationEncoderMagnetOffset.hasChanged()) {
      CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
      angleEncoder.getConfigurator().refresh(angleCANCoderConfig);
      angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();
      angleEncoder.getConfigurator().apply(angleCANCoderConfig);
    }

    shooterInputs.coastMode = !coastModeButton.get();

    // Updates Deflector Motor Inputs
    shooterInputs.deflectorMotorStatorCurrentAmps =
        deflectorMotorStatorCurrentStatusSignal.getValueAsDouble();
    shooterInputs.deflectorMotorSupplyCurrentAmps =
        deflectorMotorSupplyCurrentStatusSignal.getValueAsDouble();
    shooterInputs.deflectorMotorVoltage = deflectorMotorVoltageStatusSignal.getValueAsDouble();
    shooterInputs.deflectorMotorTemperatureCelsius =
        deflectorMotorTemperatureStatusSignal.getValueAsDouble();
  }

  @Override
  public void setShooterWheelTopVelocity(double rps) {
    shootMotorTop.setControl(shootMotorTopVelocityRequest.withVelocity(rps));
    this.topWheelVelocity = rps;
  }

  @Override
  public void setShooterWheelBottomVelocity(double rps) {
    shootMotorBottom.setControl(shootMotorBottomVelocityRequest.withVelocity(rps));
    this.bottomWheelVelocity = rps;
  }

  @Override
  public void setAngle(double angle) {
    angleMotor.setControl(angleMotorPositionRequest.withPosition(Units.degreesToRotations(angle)));
    this.angleSetpoint = angle;
  }

  @Override
  public void setAngleMotorVoltage(double voltage) {
    angleMotor.setControl(angleMotorVoltageRequest.withOutput(voltage));
  }

  @Override
  public void setDeflectorMotorVoltage(double voltage) {
    deflectorMotor.setControl(deflectorMotorVoltageRequest.withOutput(voltage));
  }

  private void configShootMotor(TalonFX shootMotor, boolean isInverted, boolean isTopMotor) {

    TalonFXConfiguration shootMotorsConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs shootMotorTorqueCurrentConfigs = new TorqueCurrentConfigs();

    if (isTopMotor) {
      shootMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent =
          ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
      shootMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent =
          -ShooterConstants.SHOOT_MOTOR_TOP_PEAK_CURRENT_LIMIT;
    } else {
      shootMotorTorqueCurrentConfigs.PeakForwardTorqueCurrent =
          ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
      shootMotorTorqueCurrentConfigs.PeakReverseTorqueCurrent =
          -ShooterConstants.SHOOT_MOTOR_BOTTOM_PEAK_CURRENT_LIMIT;
    }

    shootMotorsConfig.TorqueCurrent = shootMotorTorqueCurrentConfigs;

    if (isTopMotor) {
      shootMotorsConfig.Slot0.kP = shootMotorTopKP.get();
      shootMotorsConfig.Slot0.kI = shootMotorTopKI.get();
      shootMotorsConfig.Slot0.kD = shootMotorTopKD.get();
      shootMotorsConfig.Slot0.kS = shootMotorTopKS.get();

    } else {
      shootMotorsConfig.Slot0.kP = shootMotorBottomKP.get();
      shootMotorsConfig.Slot0.kI = shootMotorBottomKI.get();
      shootMotorsConfig.Slot0.kD = shootMotorBottomKD.get();
      shootMotorsConfig.Slot0.kS = shootMotorBottomKS.get();
    }

    shootMotorsConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    shootMotorsConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SHOOT_MOTORS_GEAR_RATIO;

    shootMotorsConfig.MotorOutput.Inverted =
        isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shootMotor.getConfigurator().apply(shootMotorsConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance()
        .registerHardware(SUBSYSTEM_NAME, isTopMotor ? "TopMotor" : "BottomMotor", shootMotor);
  }

  private void configAngleMotor(TalonFX angleMotor, CANcoder angleEncoder) {
    CANcoderConfiguration angleCANCoderConfig = new CANcoderConfiguration();
    angleCANCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    angleCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    angleCANCoderConfig.MagnetSensor.MagnetOffset = rotationEncoderMagnetOffset.get();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = angleEncoder.getConfigurator().apply(angleCANCoderConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

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

    SoftwareLimitSwitchConfigs angleMotorLimitSwitches = angleMotorConfig.SoftwareLimitSwitch;
    angleMotorLimitSwitches.ForwardSoftLimitEnable = true;
    angleMotorLimitSwitches.ForwardSoftLimitThreshold =
        Units.degreesToRotations(ShooterConstants.UPPER_ANGLE_LIMIT);
    angleMotorLimitSwitches.ReverseSoftLimitEnable = true;
    angleMotorLimitSwitches.ReverseSoftLimitThreshold =
        Units.degreesToRotations(ShooterConstants.SHOOTER_STORAGE_ANGLE);

    angleMotorConfig.Feedback.FeedbackRemoteSensorID = angleEncoder.getDeviceID();
    angleMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    angleMotorConfig.Feedback.SensorToMechanismRatio = ShooterConstants.SENSOR_TO_MECHANISM_RATIO;
    angleMotorConfig.Feedback.RotorToSensorRatio = ShooterConstants.ANGLE_MOTOR_GEAR_RATIO;

    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = angleMotor.getConfigurator().apply(angleMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleMotor", angleMotor);
    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "AngleCANcoder", angleEncoder);
  }

  public void configDeflectorMotor() {
    TalonFXConfiguration deflectorMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs deflectorMotorCurrentLimits = new CurrentLimitsConfigs();
    deflectorMotorCurrentLimits.SupplyCurrentLimit =
        ShooterConstants.DEFLECTOR_MOTOR_CONTINUOUS_CURRENT_LIMIT;
    deflectorMotorCurrentLimits.SupplyCurrentThreshold =
        ShooterConstants.DEFLECTOR_MOTOR_PEAK_CURRENT_LIMIT;
    deflectorMotorCurrentLimits.SupplyTimeThreshold =
        ShooterConstants.DEFLECTOR_MOTOR_PEAK_CURRENT_DURATION;
    deflectorMotorCurrentLimits.SupplyCurrentLimitEnable = true;
    deflectorMotorConfig.CurrentLimits = deflectorMotorCurrentLimits;

    deflectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    deflectorMotorConfig.MotorOutput.Inverted =
        ShooterConstants.DEFLECTOR_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = deflectorMotor.getConfigurator().apply(deflectorMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "DeflectorMotor", deflectorMotor);
  }

  @Override
  public void setCoastMode(boolean coast) {
    MotorOutputConfigs angleMotorConfig = new MotorOutputConfigs();
    angleMotor.getConfigurator().refresh(angleMotorConfig);
    angleMotorConfig.NeutralMode = coast ? NeutralModeValue.Coast : NeutralModeValue.Brake;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = angleMotor.getConfigurator().apply(angleMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }
  }
}
