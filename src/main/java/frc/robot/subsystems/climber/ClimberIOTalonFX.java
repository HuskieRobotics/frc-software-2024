package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private CoreCANcoder leftEncoder;
  private CoreCANcoder rightEncoder;

  private MotionMagicExpoTorqueCurrentFOC leftPositionCurrentRequest;
  private TorqueCurrentFOC leftSupplyAmpsRequest;

  private MotionMagicExpoTorqueCurrentFOC rightPositionCurrentRequest;
  private TorqueCurrentFOC rightSupplyAmpsRequest;

  private StatusSignal<Double> leftVelocityRPSStatusSignal;
  private StatusSignal<Double> leftPositionRotationsStatusSignal;
  private StatusSignal<Double> leftStatorCurrentAmpsStatusSignal;
  private StatusSignal<Double> leftSetpointStatusSignal;
  private StatusSignal<Double> leftSupplyCurrentAmpsStatusSignal;

  private StatusSignal<Double> rightVelocityRPSStatusSignal;
  private StatusSignal<Double> rightPositionMetersStatusSignal;
  private StatusSignal<Double> rightStatorCurrentAmpsStatusSignal;
  private StatusSignal<Double> rightSetpointStatusSignal;
  private StatusSignal<Double> rightSupplyCurrentAmpsStatusSignal;

  private Alert configAlert =
      new Alert("Failed to apply configuration for Climber.", AlertType.ERROR);

  // FIXME: Add KV, KA,
  private final TunableNumber kP = new TunableNumber("Climber/KP", KP);
  private final TunableNumber kI = new TunableNumber("Climber/KI", KI);
  private final TunableNumber kD = new TunableNumber("Climber/KD", KD);

  private final TunableNumber kS = new TunableNumber("Climber/KS", KS);
  private final TunableNumber kV = new TunableNumber("Climber/KV", KV);
  private final TunableNumber kA = new TunableNumber("Climber/KA", KA);
  private final TunableNumber kVExpo = new TunableNumber("Climber/KVExpo", KV_EXPO);
  private final TunableNumber kAExpo = new TunableNumber("Climber/KAExpo", KA_EXPO);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ClimberIOTalonFX() {
    configMotors(leftMotor, rightMotor);

    leftVelocityRPSStatusSignal = leftMotor.getVelocity();
    leftPositionRotationsStatusSignal = leftMotor.getPosition();
    leftStatorCurrentAmpsStatusSignal = leftMotor.getStatorCurrent();
    leftSetpointStatusSignal = leftMotor.getClosedLoopReference();

    rightVelocityRPSStatusSignal = rightMotor.getVelocity();
    rightPositionMetersStatusSignal = rightMotor.getPosition();
    rightStatorCurrentAmpsStatusSignal = rightMotor.getStatorCurrent();
    rightSetpointStatusSignal = rightMotor.getClosedLoopReference();

    leftPositionCurrentRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    rightPositionCurrentRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
  }

  /**
   * Update the inputs based on the current state of the TalonFX leftMotor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ClimberIOInputs inputs) { // FIXME: May need inputs for both motors
    BaseStatusSignal.refreshAll(
        leftVelocityRPSStatusSignal,
        leftPositionRotationsStatusSignal,
        leftStatorCurrentAmpsStatusSignal,
        leftSetpointStatusSignal,
        leftSupplyCurrentAmpsStatusSignal,
        rightVelocityRPSStatusSignal,
        rightPositionMetersStatusSignal,
        rightStatorCurrentAmpsStatusSignal,
        rightSetpointStatusSignal,
        rightSupplyCurrentAmpsStatusSignal);

    inputs.leftVelocityRPS = leftVelocityRPSStatusSignal.getValueAsDouble();
    inputs.leftPositionMeters =
        leftPositionRotationsStatusSignal.getValueAsDouble() * DRUM_CIRCUMFERENCE;
    inputs.leftStatorCurrentAmps = leftStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftSetpoint = leftSetpointStatusSignal.getValueAsDouble();

    inputs.rightVelocityRPS = rightVelocityRPSStatusSignal.getValueAsDouble();
    inputs.rightPositionMeters = rightPositionMetersStatusSignal.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightSetpoint = rightSetpointStatusSignal.getValueAsDouble();

    // inputs.closedLoopError = leftMotor.getClosedLoopError().getValue();
    // inputs.power =
    // inputs.controlMode = leftMotor.getControlMode().toString();

    // update configuration if tunables have changed
    if (kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kS.hasChanged()
        || kA.hasChanged()
        || kV.hasChanged()
        || kVExpo.hasChanged()
        || kAExpo.hasChanged()) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

      this.leftMotor.getConfigurator().refresh(config);
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kI.get();
      config.Slot0.kD = kD.get();
      config.Slot0.kS = kS.get();
      config.Slot0.kV = kV.get();
      config.Slot0.kA = kA.get();
      motionMagicConfigs.MotionMagicExpo_kV = kVExpo.get();
      motionMagicConfigs.MotionMagicExpo_kA = kAExpo.get();

      this.leftMotor.getConfigurator().apply(config);
      this.rightMotor.getConfigurator().apply(config);
    }
  }

  /**
   * Set the leftMotor position to the specified value in degrees.
   *
   * @param position the position to set the leftMotor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setLeftMotorPosition(double position) {
    leftMotor.setControl(
        leftPositionCurrentRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(KG));
  }

  @Override
  public void setRightMotorPosition(double position) {
    this.rightMotor.setControl(
        rightPositionCurrentRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(KG));
  }

  @Override
  public void setLeftMotorPower(double current) {
    leftMotor.setControl(leftSupplyAmpsRequest.withOutput(current));
  }

  @Override
  public void setRightMotorPower(double current) {
    rightMotor.setControl(rightSupplyAmpsRequest.withOutput(current));
  }

  @Override
  public void setPositionZero() {
    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);
  }

  private void configMotors(TalonFX leftMotor, TalonFX rightMotor) {
    this.leftMotor = new TalonFX(LEFT_MOTOR_CAN_ID, RobotConfig.getInstance().getCANBusName());
    this.rightMotor = new TalonFX(RIGHT_MOTOR_CAN_ID, RobotConfig.getInstance().getCANBusName());

    this.leftEncoder = new CoreCANcoder(LEFT_MOTOR_CAN_ID);
    this.rightEncoder = new CoreCANcoder(RIGHT_MOTOR_CAN_ID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    StatusCode rightStatus = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.leftMotor.getConfigurator().apply(config);
      rightStatus = this.rightMotor.getConfigurator().apply(config);
      if (status.isOK() && rightStatus.isOK()) {
        configAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    setPositionZero();
    FaultReporter.getInstance().registerHardware(CLIMBER, "Climber leftMotor", leftMotor);
    FaultReporter.getInstance().registerHardware(CLIMBER, "Climber rightMotor", rightMotor);
  }
}
