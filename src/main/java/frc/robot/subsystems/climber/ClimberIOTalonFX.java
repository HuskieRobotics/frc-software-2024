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
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Current;
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

  private MotionMagicExpoTorqueCurrentFOC leftPositionCurrentRequest;
  private TorqueCurrentFOC leftSupplyAmpsRequest;

  private MotionMagicExpoTorqueCurrentFOC rightPositionCurrentRequest;
  private TorqueCurrentFOC rightSupplyAmpsRequest;

  private double leftRequestedReferenceSetpointMeters;
  private double leftRequestedSupplyAmps;

  private double rightRequestedReferenceSetpointMeters;
  private double rightRequestedSupplyAmps;

  private StatusSignal<Double> leftVelocityRPSStatusSignal;
  private StatusSignal<Double> leftPositionMeterStatusSignal;
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
  private final TunableNumber leftMotorKP =
      new TunableNumber("Climber/leftMotorKP", LEFT_POSITION_PID_P);
  private final TunableNumber leftMotorKI =
      new TunableNumber("Climber/leftMotorKI", LEFT_POSITION_PID_I);
  private final TunableNumber leftMotorKD =
      new TunableNumber("Climber/leftMotorKD", LEFT_POSITION_PID_D);
  private final TunableNumber leftMotorKS =
      new TunableNumber("Climber/leftMotorKS", LEFT_POSITION_PID_KS);
  private final TunableNumber leftMotorKV =
      new TunableNumber("Climber/leftMotorKV", LEFT_POSITION_PID_KV);
  private final TunableNumber leftMotorKA = 
      new TunableNumber("Climber/leftMotorKA", LEFT_POSITION_PID_KA);
  private final TunableNumber leftMotorKVExpo = 
      new TunableNumber("Climber/leftMotorKVExpo", LEFT_POSITION_PID_KV_EXPO);
  private final TunableNumber leftMotorKAExpo = 
      new TunableNumber("Climber/leftMotorKAExpo", LEFT_POSITION_PID_KA_EXPO);
  

  private final TunableNumber rightMotorKP =
      new TunableNumber("Climber/rightMotorKP", RIGHT_POSITION_PID_P);
  private final TunableNumber rightMotorKI =
      new TunableNumber("Climber/rightMotorKI", RIGHT_POSITION_PID_I);
  private final TunableNumber rightMotorKD =
      new TunableNumber("Climber/rightMotorKD", RIGHT_POSITION_PID_D);
  private final TunableNumber rightMotorKS =
      new TunableNumber("Climber/rightMotorKS", RIGHT_POSITION_PID_KS);
  private final TunableNumber rightMotorKV = 
      new TunableNumber("Climber/rightMotorKV", RIGHT_POSITION_PID_KV);
  private final TunableNumber rightMotorKA = 
      new TunableNumber("Climber/rightMotorKA", RIGHT_POSITION_PID_KA);
  private final TunableNumber rightMotorKVExpo = 
      new TunableNumber("Climber/rightMotorKVExpo", RIGHT_POSITION_PID_KV_EXPO);
  private final TunableNumber rightMotorKAExpo = 
      new TunableNumber("Climber/rightMotorKAExpo", RIGHT_POSITION_PID_KA_EXPO);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ClimberIOTalonFX() {
    leftVelocityRPSStatusSignal = leftMotor.getVelocity();
    leftPositionMeterStatusSignal = leftMotor.getPosition();
    leftStatorCurrentAmpsStatusSignal = leftMotor.getStatorCurrent();
    leftSetpointStatusSignal =
        leftMotor.getClosedLoopReference(); 

    rightVelocityRPSStatusSignal = rightMotor.getVelocity();
    rightPositionMetersStatusSignal = rightMotor.getPosition();
    rightStatorCurrentAmpsStatusSignal = rightMotor.getStatorCurrent();
    rightSetpointStatusSignal = rightMotor.getClosedLoopReference();

    leftPositionCurrentRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);
    rightPositionCurrentRequest = new MotionMagicExpoTorqueCurrentFOC(0.0);

    leftConfigMotor(LEFT_MOTOR_CAN_ID);
    rightConfigMotor(RIGHT_MOTOR_CAN_ID);
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
        leftPositionMeterStatusSignal,
        leftStatorCurrentAmpsStatusSignal,
        leftSetpointStatusSignal,
        leftSupplyCurrentAmpsStatusSignal,
        rightVelocityRPSStatusSignal,
        rightPositionMetersStatusSignal,
        rightStatorCurrentAmpsStatusSignal,
        rightSetpointStatusSignal,
        rightSupplyCurrentAmpsStatusSignal);

    inputs.leftVelocityRPS = leftVelocityRPSStatusSignal.getValueAsDouble();
    inputs.leftPositionMeters = leftPositionMeterStatusSignal.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftCurrentSetpoint = leftSetpointStatusSignal.getValueAsDouble();

    inputs.rightVelocityRPS = rightVelocityRPSStatusSignal.getValueAsDouble();
    inputs.rightPositionMeters = rightPositionMetersStatusSignal.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightCurrentSetpoint = rightSetpointStatusSignal.getValueAsDouble();

    inputs.leftReferenceSetpointMeters = leftRequestedReferenceSetpointMeters;
    inputs.leftSupplyCurrentAmps = leftRequestedSupplyAmps;

    inputs.rightReferenceSetpointMeters = rightRequestedReferenceSetpointMeters;
    inputs.rightSupplyCurrentAmps = rightRequestedSupplyAmps;

    // inputs.closedLoopError = leftMotor.getClosedLoopError().getValue();
    // inputs.power =
    // inputs.controlMode = leftMotor.getControlMode().toString();

    // update configuration if tunables have changed
    if (leftMotorKP.hasChanged()
        || leftMotorKI.hasChanged()
        || leftMotorKD.hasChanged()
        || leftMotorKS.hasChanged()
        || rightMotorKP.hasChanged()
        || rightMotorKI.hasChanged()
        || rightMotorKD.hasChanged()
        || rightMotorKS.hasChanged()) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();

      this.leftMotor.getConfigurator().refresh(config);
      config.Slot0.kP = leftMotorKP.get();
      config.Slot0.kI = leftMotorKI.get();
      config.Slot0.kD = leftMotorKD.get();
      config.Slot0.kS = leftMotorKS.get();
      config.Slot0.kV = leftMotorKV.get();
      config.Slot0.kA = leftMotorKA.get();
      motionMagicConfigs.MotionMagicExpo_kV = leftMotorKVExpo.get();
      motionMagicConfigs.MotionMagicExpo_kA = leftMotorKAExpo.get();

      this.leftMotor.getConfigurator().apply(config);

      this.rightMotor.getConfigurator().refresh(config);
      config.Slot0.kP = rightMotorKP.get();
      config.Slot0.kI = rightMotorKI.get();
      config.Slot0.kD = rightMotorKD.get();
      config.Slot0.kS = rightMotorKS.get();
      config.Slot0.kV = rightMotorKV.get();
      config.Slot0.kA = rightMotorKA.get();
      motionMagicConfigs.MotionMagicExpo_kV = rightMotorKVExpo.get();
      motionMagicConfigs.MotionMagicExpo_kA = rightMotorKAExpo.get();
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
            .withFeedForward(POSITION_PID_KG)); 
  }

  @Override
  public void setRightMotorPosition(double position) {
    this.rightMotor.setControl(
        rightPositionCurrentRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(POSITION_PID_KG));
  }

  @Override
  public void setLeftMotorPower(double current) {
    leftMotor.setControl(leftSupplyAmpsRequest.withOutput(current)); // FIXME: Is this how we set current?
    leftRequestedSupplyAmps = current;
  }

  @Override
  public void setRightMotorPower(double current) {
    rightMotor.setControl(leftSupplyAmpsRequest.withOutput(current));    
    rightRequestedSupplyAmps = current;
  }

  private void leftConfigMotor(int leftMotorID) {
    this.leftMotor = new TalonFX(leftMotorID, RobotConfig.getInstance().getCANBusName());

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
    config.Slot0.kP = leftMotorKP.get();
    config.Slot0.kI = leftMotorKI.get();
    config.Slot0.kD = leftMotorKD.get();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.leftMotor.getConfigurator().apply(config);
      if (status.isOK()) {
        configAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    this.leftMotor.setPosition(0);

    FaultReporter.getInstance().registerHardware(CLIMBER, "Climber leftMotor", leftMotor);
  }

  private void rightConfigMotor(int rightMotorID) {
    this.rightMotor = new TalonFX(rightMotorID, RobotConfig.getInstance().getCANBusName());

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
    config.Slot0.kP = rightMotorKP.get();
    config.Slot0.kI = rightMotorKI.get();
    config.Slot0.kD = rightMotorKD.get();

    config.Voltage.PeakForwardVoltage = rightMotorKS.get();
    config.Voltage.PeakReverseVoltage = rightMotorKS.get();

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = this.rightMotor.getConfigurator().apply(config);
      if (status.isOK()) {
        configAlert.set(false);
        break;
      }
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    this.rightMotor.setPosition(0);

    FaultReporter.getInstance().registerHardware(CLIMBER, "Climber rightMotor", rightMotor);
  }
}
