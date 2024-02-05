package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

  private PositionTorqueCurrentFOC leftPositionCurrentRequest;
  private VelocityTorqueCurrentFOC leftVelocityRequest;

  private PositionTorqueCurrentFOC rightPositionCurrentRequest;
  private VelocityTorqueCurrentFOC rightVelocityRequest;

  private double leftRequestedVelocity;
  private double leftRequestedSetpoint; 

  private double rightRequestedVelocity;
  private double rightRequestedSetpoint;

  private StatusSignal<Double> leftVelocityRPMStatusSignal;
  private StatusSignal<Double> leftPositionStatusSignal;
  private StatusSignal<Double> leftStatorCurrentAmpsStatusSignal;
  private StatusSignal<Double> leftSetpointStatusSignal;
  private StatusSignal<Double> leftVoltageRequest;

  private StatusSignal<Double> rightVelocityRPMStatusSignal;
  private StatusSignal<Double> rightPositionStatusSignal;
  private StatusSignal<Double> rightStatorCurrentAmpsStatusSignal;
  private StatusSignal<Double> rightSetpointStatusSignal;

  private Alert configAlert =
      new Alert("Failed to apply configuration for Climber.", AlertType.ERROR);

  private final TunableNumber leftMotorKP = new TunableNumber("Climber/leftMotorKP", LEFT_POSITION_PID_P);
  private final TunableNumber leftMotorKI = new TunableNumber("Climber/leftMotorKI", LEFT_POSITION_PID_I);
  private final TunableNumber leftMotorKD = new TunableNumber("Climber/leftMotorKD", LEFT_POSITION_PID_D);
  private final TunableNumber leftMotorKS =
      new TunableNumber("Climber/leftMotorKS", LEFT_POSITION_PID_KS);

  private final TunableNumber rightMotorKP = new TunableNumber("Climber/rightMotorKP", RIGHT_POSITION_PID_P);
  private final TunableNumber rightMotorKI = new TunableNumber("Climber/rightMotorKI", RIGHT_POSITION_PID_I);
  private final TunableNumber rightMotorKD = new TunableNumber("Climber/rightMotorKD", RIGHT_POSITION_PID_D);
  private final TunableNumber rightMotorKS =
      new TunableNumber("Climber/rightMotorKS", RIGHT_POSITION_PID_KS);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ClimberIOTalonFX() {
    leftVelocityRPMStatusSignal = leftMotor.getRotorVelocity();
    leftPositionStatusSignal = leftMotor.getPosition();
    leftStatorCurrentAmpsStatusSignal = leftMotor.getStatorCurrent();
    leftSetpointStatusSignal = leftMotor.getClosedLoopReference(); // FIXME: Is closedLoopReference the setpoint?
    leftVoltageRequest = leftMotor.getMotorVoltage();

    rightVelocityRPMStatusSignal = rightMotor.getPosition();
    rightPositionStatusSignal = rightMotor.getPosition();
    rightStatorCurrentAmpsStatusSignal = rightMotor.getStatorCurrent();
    rightSetpointStatusSignal = rightMotor.getClosedLoopReference();
    
    leftVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    leftPositionCurrentRequest = new PositionTorqueCurrentFOC(0.0);

    rightVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
    rightPositionCurrentRequest = new PositionTorqueCurrentFOC(0.0);

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
      leftVelocityRPMStatusSignal,
      leftPositionStatusSignal,
      leftStatorCurrentAmpsStatusSignal,
      leftSetpointStatusSignal,
      rightVelocityRPMStatusSignal,
      rightPositionStatusSignal,
      rightStatorCurrentAmpsStatusSignal,
      rightSetpointStatusSignal
    );

    inputs.leftVelocityRPM = leftVelocityRPMStatusSignal.getValueAsDouble();
    inputs.leftPosition = leftPositionStatusSignal.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.leftSetpoint = leftSetpointStatusSignal.getValueAsDouble();

    inputs.rightVelocityRPM = rightVelocityRPMStatusSignal.getValueAsDouble();
    inputs.rightPosition = rightPositionStatusSignal.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrentAmpsStatusSignal.getValueAsDouble();
    inputs.rightSetpoint = rightSetpointStatusSignal.getValueAsDouble();

    inputs.leftReferenceVelocity = leftRequestedVelocity;
    inputs.leftReferenceSetpoint = leftRequestedSetpoint;

    inputs.rightReferenceVelocity = rightRequestedVelocity;
    inputs.rightReferenceSetpoint = rightRequestedSetpoint;

    // inputs.closedLoopError = leftMotor.getClosedLoopError().getValue();
    // inputs.power = 
    // inputs.controlMode = leftMotor.getControlMode().toString();

    // update configuration if tunables have changed
    if (leftMotorKP.hasChanged() || leftMotorKI.hasChanged() || leftMotorKD.hasChanged() || leftMotorKS.hasChanged() || rightMotorKP.hasChanged() || rightMotorKI.hasChanged() || rightMotorKD.hasChanged() || rightMotorKS.hasChanged()) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      this.leftMotor.getConfigurator().refresh(config);
      config.Slot0.kP = leftMotorKP.get();
      config.Slot0.kI = leftMotorKI.get();
      config.Slot0.kD = leftMotorKD.get();
      config.TorqueCurrent.PeakForwardTorqueCurrent = leftMotorKS.get(); // FIXME: Is this the right way to set the peak forward torque current?
      config.TorqueCurrent.PeakReverseTorqueCurrent = leftMotorKS.get();
      this.leftMotor.getConfigurator().apply(config);

      this.rightMotor.getConfigurator().refresh(config);
      config.Slot0.kP = rightMotorKP.get();
      config.Slot0.kI = rightMotorKI.get();
      config.Slot0.kD = rightMotorKD.get();
      config.TorqueCurrent.PeakForwardTorqueCurrent = rightMotorKS.get();
      config.TorqueCurrent.PeakReverseTorqueCurrent = rightMotorKS.get();
      this.rightMotor.getConfigurator().apply(config);
    }
  }

  /**
   * Set the leftMotor current to the specified value in amps.x
   *
   * @param rps the rotations per second to set the leftMotor
   */
  @Override
  public void setLeftMotorVelocity(double rps) {
    this.leftMotor.setControl(leftVelocityRequest.withVelocity(rps));
    this.leftRequestedVelocity = rps;
  }

  /**
   * Set the leftMotor position to the specified value in degrees.
   *
   * @param position the position to set the leftMotor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setLeftMotorPosition(double position, double arbitraryFeedForward) {
    leftMotor.setControl(
        leftPositionCurrentRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }
  
  @Override
  public void setRightMotorVelocity(double rps) {
    this.rightMotor.setControl(rightVelocityRequest.withVelocity(rps));
    this.rightRequestedVelocity = rps;
  }

  @Override
  public void setRightMotorPosition(double position, double arbitraryFeedForward) {
    this.rightMotor.setControl(
        rightPositionCurrentRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
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

    config.MotorOutput.Inverted =
        MOTOR_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = leftMotorKP.get();
    config.Slot0.kI = leftMotorKI.get();
    config.Slot0.kD = leftMotorKD.get();

    config.Voltage.PeakForwardVoltage = leftMotorKS.get();
    config.Voltage.PeakReverseVoltage = leftMotorKS.get();

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
