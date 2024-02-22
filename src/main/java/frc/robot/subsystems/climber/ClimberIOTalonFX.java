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
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

/** TalonFX implementation of the generic SubsystemIO */
public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private MotionMagicExpoTorqueCurrentFOC leftPositionCurrentRequest;
  private TorqueCurrentFOC leftSupplyAmpsRequest;

  private MotionMagicExpoTorqueCurrentFOC rightPositionCurrentRequest;
  private TorqueCurrentFOC rightSupplyAmpsRequest;

  private boolean longArms;

  private FaultReporter faultReporter;

  private StatusSignal<Double> leftVelocityRPSStatusSignal;
  private StatusSignal<Double> leftPositionRotationsStatusSignal;
  private StatusSignal<Double> leftStatorCurrentAmpsStatusSignal;
  private StatusSignal<Double> leftSetpointStatusSignal;
  private StatusSignal<Double> leftSupplyCurrentAmpsStatusSignal;

  private StatusSignal<Double> rightVelocityRPSStatusSignal;
  private StatusSignal<Double> rightPositionRotationsStatusSignal;
  private StatusSignal<Double> rightStatorCurrentAmpsStatusSignal;
  private StatusSignal<Double> rightSetpointStatusSignal;
  private StatusSignal<Double> rightSupplyCurrentAmpsStatusSignal;

  private Alert configAlert =
      new Alert("Failed to apply configuration for Climber.", AlertType.ERROR);

  private final TunableNumber kP = new TunableNumber("Climber/KP", KP);
  private final TunableNumber kI = new TunableNumber("Climber/KI", KI);
  private final TunableNumber kD = new TunableNumber("Climber/KD", KD);

  private final TunableNumber kS = new TunableNumber("Climber/KS", KS);
  private final TunableNumber kV = new TunableNumber("Climber/KV", KV);
  private final TunableNumber kA = new TunableNumber("Climber/KA", KA);
  private final TunableNumber kVExpo = new TunableNumber("Climber/KVExpo", KV_EXPO);
  private final TunableNumber kAExpo = new TunableNumber("Climber/KAExpo", KA_EXPO);
  private final TunableNumber kG = new TunableNumber("Climber/KG", KG);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ClimberIOTalonFX() {
    leftMotor = configMotors(LEFT_MOTOR_CAN_ID, LEFT_MOTOR_INVERTED, faultReporter);
    rightMotor = configMotors(RIGHT_MOTOR_CAN_ID, RIGHT_MOTOR_INVERTED, faultReporter);

    leftVelocityRPSStatusSignal = leftMotor.getVelocity();
    leftPositionRotationsStatusSignal = leftMotor.getPosition();
    leftStatorCurrentAmpsStatusSignal = leftMotor.getStatorCurrent();
    leftSetpointStatusSignal = leftMotor.getClosedLoopReference();
    leftSupplyCurrentAmpsStatusSignal = leftMotor.getSupplyCurrent();

    rightVelocityRPSStatusSignal = rightMotor.getVelocity();
    rightPositionRotationsStatusSignal = rightMotor.getPosition();
    rightStatorCurrentAmpsStatusSignal = rightMotor.getStatorCurrent();
    rightSetpointStatusSignal = rightMotor.getClosedLoopReference();
    rightSupplyCurrentAmpsStatusSignal = rightMotor.getSupplyCurrent();

    leftPositionCurrentRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    rightPositionCurrentRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);

    leftSupplyAmpsRequest = new TorqueCurrentFOC(0.0);
    rightSupplyAmpsRequest = new TorqueCurrentFOC(0.0);

    longArms = false;
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
        rightPositionRotationsStatusSignal,
        rightStatorCurrentAmpsStatusSignal,
        rightSetpointStatusSignal,
        rightSupplyCurrentAmpsStatusSignal);

    inputs.leftVelocityRPS = leftVelocityRPSStatusSignal.getValueAsDouble();
    inputs.leftPositionRotationsMeters =
        leftPositionRotationsStatusSignal.getValueAsDouble() * DRUM_CIRCUMFERENCE_METERS;
    inputs.leftStatorCurrentAmps = leftStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftSetpoint = leftSetpointStatusSignal.getValueAsDouble();

    inputs.rightVelocityRPS = rightVelocityRPSStatusSignal.getValueAsDouble();
    inputs.rightPositionRotationsMeters =
        rightPositionRotationsStatusSignal.getValueAsDouble() * DRUM_CIRCUMFERENCE_METERS;
    inputs.rightStatorCurrentAmps = rightStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightSetpoint = rightSetpointStatusSignal.getValueAsDouble();

    // update configuration if tunables have changed
    if (kP.hasChanged()
        || kI.hasChanged()
        || kD.hasChanged()
        || kS.hasChanged()
        || kA.hasChanged()
        || kV.hasChanged()
        || kVExpo.hasChanged()
        || kAExpo.hasChanged()
        || kG.hasChanged()) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      MotionMagicConfigs motionMagicConfigs = config.MotionMagic;

      this.leftMotor.getConfigurator().refresh(config);
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kI.get();
      config.Slot0.kD = kD.get();
      config.Slot0.kS = kS.get();
      config.Slot0.kV = kV.get();
      config.Slot0.kA = kA.get();
      motionMagicConfigs.MotionMagicExpo_kV = kVExpo.get();
      motionMagicConfigs.MotionMagicExpo_kA = kAExpo.get();
      config.Slot0.kG = kG.get();

      this.leftMotor.getConfigurator().apply(config);
      this.rightMotor.getConfigurator().refresh(config);
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kI.get();
      config.Slot0.kD = kD.get();
      config.Slot0.kS = kS.get();
      config.Slot0.kV = kV.get();
      config.Slot0.kA = kA.get();
      motionMagicConfigs.MotionMagicExpo_kV = kVExpo.get();
      motionMagicConfigs.MotionMagicExpo_kA = kAExpo.get();
      config.Slot0.kG = kG.get();
      this.rightMotor.getConfigurator().apply(config);
    }
  }

  /**
   * Set the leftMotor position to the specified value in degrees.
   *
   * @param position the position to set the leftMotor to in meters
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setLeftMotorPosition(double position) {
    leftMotor.setControl(
        leftPositionCurrentRequest
            .withPosition(position / DRUM_CIRCUMFERENCE_METERS)
            .withFeedForward(KG));
  }

  @Override
  public void setRightMotorPosition(double position) {
    leftMotor.setControl(
        rightPositionCurrentRequest
            .withPosition(position / DRUM_CIRCUMFERENCE_METERS)
            .withFeedForward(KG));
  }

  @Override
  public void setLeftMotorCurrent(double current) {
    leftMotor.setControl(leftSupplyAmpsRequest.withOutput(current));
  }

  @Override
  public void setRightMotorCurrent(double current) {
    rightMotor.setControl(rightSupplyAmpsRequest.withOutput(current));
  }

  @Override
  public void setPositionZero() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  @Override
  public void enableLongArms() {
    longArms = true;
  }

  @Override
  public void disableLongArms() {
    longArms = false;
  }

  @Override
  public boolean getLongArms() {
    return longArms;
  }

  private TalonFX configMotors(int canID, boolean isInverted, FaultReporter fault) {
    TalonFX motor = new TalonFX(canID, RobotConfig.getInstance().getCANBusName());
    motor.setInverted(isInverted);

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs();
    currentLimits.SupplyCurrentLimit = CONTINUOUS_CURRENT_LIMIT;
    currentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
    currentLimits.SupplyCurrentThreshold = PEAK_CURRENT_LIMIT;
    currentLimits.SupplyTimeThreshold = PEAK_CURRENT_DURATION;
    currentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits = currentLimits;

    config.Feedback.SensorToMechanismRatio = GEAR_RATIO;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();
    config.Slot0.kS = kS.get();
    config.Slot0.kV = kV.get();
    config.Slot0.kA = kA.get();
    config.Slot0.kG = kG.get();

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
    fault.getInstance().registerHardware(CLIMBER, "Climber motor", motor);
    return motor;
  }
}
