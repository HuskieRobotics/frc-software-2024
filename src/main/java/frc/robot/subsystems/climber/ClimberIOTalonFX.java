package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private Alert configAlert =
      new Alert("Failed to apply configuration for Climber.", AlertType.ERROR);

  private final TunableNumber kP = new TunableNumber("Climber/kP", POSITION_PID_P);
  private final TunableNumber kI = new TunableNumber("Climber/kI", POSITION_PID_I);
  private final TunableNumber kD = new TunableNumber("Climber/kD", POSITION_PID_D);
  private final TunableNumber kPeakOutput =
      new TunableNumber("Climber/kPeakOutput", POSITION_PID_PEAK_OUTPUT);

  /** Create a TalonFX-specific generic SubsystemIO */
  public ClimberIOTalonFX() {
    configMotor(LEFT_MOTOR_CAN_ID, RIGHT_MOTOR_CAN_ID);
  }

  /**
   * Update the inputs based on the current state of the TalonFX leftMotor controller.
   *
   * @param inputs the inputs object to update
   */
  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.positionDeg =
        Conversions.falconRotationsToMechanismDegrees(
            leftMotor.getRotorPosition().getValue(), GEAR_RATIO);
    inputs.velocityRPM =
        Conversions.falconRPSToMechanismRPM(leftMotor.getRotorVelocity().getValue(), GEAR_RATIO);
    inputs.closedLoopError = leftMotor.getClosedLoopError().getValue();
    inputs.setpoint = leftMotor.getClosedLoopReference().getValue();
    inputs.power = leftMotor.getDutyCycle().getValue();
    inputs.controlMode = leftMotor.getControlMode().toString();
    inputs.statorCurrentAmps = leftMotor.getStatorCurrent().getValue();
    inputs.tempCelsius = leftMotor.getDeviceTemp().getValue();
    inputs.supplyCurrentAmps = leftMotor.getSupplyCurrent().getValue();

    // update configuration if tunables have changed
    if (kP.hasChanged() || kI.hasChanged() || kD.hasChanged() || kPeakOutput.hasChanged()) {
      TalonFXConfiguration config = new TalonFXConfiguration();
      this.leftMotor.getConfigurator().refresh(config);
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kI.get();
      config.Slot0.kD = kD.get();
      config.Voltage.PeakForwardVoltage = kPeakOutput.get();
      config.Voltage.PeakReverseVoltage = kPeakOutput.get();
      this.leftMotor.getConfigurator().apply(config);
    }
  }

  // FIXME: Replace voltage requests 
  /**
   * Set the leftMotor power to the specified percentage of maximum power.
   *
   * @param power the percentage of maximum power to set the leftMotor to
   */
  @Override
  public void setLeftMotorPower(double power) {
    this.leftMotor.setControl(voltageRequest.withOutput(power * 12.0));
  }

  /**
   * Set the leftMotor current to the specified value in amps.
   *
   * @param power the current to set the leftMotor to in amps
   */
  @Override
  public void setLeftMotorCurrent(double current) {
    this.leftMotor.setControl(currentRequest.withOutput(current));
  }

  /**
   * Set the leftMotor position to the specified value in degrees.
   *
   * @param position the position to set the leftMotor to in degrees
   * @param arbitraryFeedForward the arbitrary feed forward as a percentage of maximum power
   */
  @Override
  public void setLeftMotorPosition(double position, double arbitraryFeedForward) {
    this.leftMotor.setControl(
        positionRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  // Methods for right motor below
  @Override
  public void setRightMotorPower(double power) {
    this.rightMotor.setControl(voltageRequest.withOutput(power * 12.0));
  }
  
  @Override
  public void setRightMotorCurrent(double current) {
    this.rightMotor.setControl(currentRequest.withOutput(current));
  }

  @Override
  public void setRightMotorPosition(double position, double arbitraryFeedForward) {
    this.rightMotor.setControl(
        positionRequest
            .withPosition(Conversions.degreesToFalconRotations(position, GEAR_RATIO))
            .withFeedForward(arbitraryFeedForward));
  }

  private void configMotor(int leftMotorID, int rightMotorID) {

    this.leftMotor = new TalonFX(leftMotorID, RobotConfig.getInstance().getCANBusName());
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
    config.Slot0.kP = kP.get();
    config.Slot0.kI = kI.get();
    config.Slot0.kD = kD.get();

    config.Voltage.PeakForwardVoltage = kPeakOutput.get();
    config.Voltage.PeakReverseVoltage = kPeakOutput.get();

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
    this.rightMotor.setPosition(0);

    // FIXME: May need to log the right motor.
    this.voltageRequest = new VoltageOut(0.0);
    this.voltageRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();
    this.currentRequest = new TorqueCurrentFOC(0.0);
    this.positionRequest = new PositionVoltage(0.0).withSlot(0);
    this.positionRequest.EnableFOC = RobotConfig.getInstance().getPhoenix6Licensed();

    FaultReporter.getInstance().registerHardware(CLIMBER, "Climber leftMotor", leftMotor);
    FaultReporter.getInstance().registerHardware(CLIMBER, "Climber rightMotor", rightMotor);
  }
}
