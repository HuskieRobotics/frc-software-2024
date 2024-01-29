package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team6328.util.TunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rightRollerMotor;
  private TalonFX leftRollerMotor;
  private TalonFX drumMotor;
  private final DigitalInput rightRollerIRSensor;
  private final DigitalInput leftRollerIRSensor;
  private final DigitalInput drumIRSensor;

  private VelocityTorqueCurrentFOC rightRollerVelocityRequest;
  private VelocityTorqueCurrentFOC leftRollerVelocityRequest;
  private VelocityTorqueCurrentFOC drumVelocityRequest;

  private StatusSignal<Double> rightRollerVelocityStatusSignal;
  private StatusSignal<Double> leftRollerVelocityStatusSignal;
  private StatusSignal<Double> drumVelocityStatusSignal;
  // status signals for stator current
  private StatusSignal<Double> rightRollerStatorCurrentStatusSignal;
  private StatusSignal<Double> leftRollerStatorCurrentStatusSignal;
  private StatusSignal<Double> drumStatorCurrentStatusSignal;

  // tunable numbers for roller and drum pid
  private final TunableNumber rollerMotorsKP = 
      new TunableNumber("Intake/rollerMotorsKP", IntakeConstants.INTAKE_ROLLER_MOTORS_KP);

  private final TunableNumber rollerMotorsKI =
      new TunableNumber("Intake/rollerMotorsKI", IntakeConstants.INTAKE_ROLLER_MOTORS_KI);

  private final TunableNumber rollerMotorsKD =
      new TunableNumber("Intake/rollerMotorsKD", IntakeConstants.INTAKE_ROLLER_MOTORS_KD);

  private final TunableNumber drumMotorKP =
      new TunableNumber("Intake/drumMotorKP", IntakeConstants.INTAKE_DRUM_MOTOR_KP);
  private final TunableNumber drumMotorKI =
      new TunableNumber("Intake/drumMotorKI", IntakeConstants.INTAKE_DRUM_MOTOR_KI);
  private final TunableNumber drumMotorKD = 
      new TunableNumber("Intake/drumMotorKD", IntakeConstants.INTAKE_DRUM_MOTOR_KD);


  public IntakeIOTalonFX() {
    rightRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_RIGHT_ROLLER_IR_SENSOR_ID);
    leftRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_LEFT_ROLLER_IR_SENSOR_ID);
    drumIRSensor = new DigitalInput(IntakeConstants.INTAKE_DRUM_IR_SENSOR_ID);

    configureIntakeMotors();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isRightRollerIRBlocked = rightRollerIRSensor.get();
    inputs.isLeftRollerIRBlocked = leftRollerIRSensor.get();
    inputs.isDrumIRBlocked = drumIRSensor.get();

    inputs.rightRollerStatorCurrentAmps = rightRollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.leftRollerStatorCurrentAmps = leftRollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.drumStatorCurrentAmps = drumStatorCurrentStatusSignal.getValueAsDouble();

    inputs.rightRollerVelocityRotationsPerSecond = rightRollerVelocityStatusSignal.getValueAsDouble();
    inputs.leftRollerVelocityRotationsPerSecond = leftRollerVelocityStatusSignal.getValueAsDouble();
    inputs.drumVelocityRotationsPerSecond = drumVelocityStatusSignal.getValueAsDouble();

    // what would I use here for reference velocity?
  }

  @Override
  public void setRightRollerVelocity(double rps) {
    rightRollerMotor.setControl(rightRollerVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setLeftRollerVelocity(double rps) {
    leftRollerMotor.setControl(leftRollerVelocityRequest.withVelocity(rps));
  }

  public void setDrumVelocity(double rps) {
    drumMotor.setControl(drumVelocityRequest.withVelocity(rps));
  }

  private void configureIntakeMotors() {
    rightRollerMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_ID);
    leftRollerMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_ID);
    drumMotor = new TalonFX(IntakeConstants.INTAKE_DRUM_MOTOR_ID);

    rightRollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    leftRollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    drumVelocityRequest = new VelocityTorqueCurrentFOC(0);

    
    // assuming rotorvelocity because in tunerx this is what was used to retrieve rps
    rightRollerVelocityStatusSignal = rightRollerMotor.getRotorVelocity();
    leftRollerVelocityStatusSignal = leftRollerMotor.getRotorVelocity();
    drumVelocityStatusSignal = drumMotor.getRotorVelocity();
    
    rightRollerStatorCurrentStatusSignal = rightRollerMotor.getStatorCurrent();
    leftRollerStatorCurrentStatusSignal = leftRollerMotor.getStatorCurrent();
    drumStatorCurrentStatusSignal = drumMotor.getStatorCurrent();

    TalonFXConfiguration rightRollerConfig = new TalonFXConfiguration();
    TalonFXConfiguration leftRollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();

    rollerCurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLERS_CONTINUOUS_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyCurrentThreshold = IntakeConstants.ROLLERS_PEAK_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyTimeThreshold = IntakeConstants.ROLLERS_PEAK_CURRENT_DURATION;
    rollerCurrentLimits.SupplyCurrentLimitEnable = true;

    rightRollerConfig.CurrentLimits = rollerCurrentLimits;
    leftRollerConfig.CurrentLimits = rollerCurrentLimits;

    rightRollerConfig.MotorOutput.Inverted =
        IntakeConstants.ROLLERS_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    leftRollerConfig.MotorOutput.Inverted =
        IntakeConstants.ROLLERS_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    rightRollerMotor.getConfigurator().apply(rightRollerConfig);
    leftRollerMotor.getConfigurator().apply(leftRollerConfig);


    TalonFXConfiguration drumConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs drumCurrentLimits = new CurrentLimitsConfigs();

    drumCurrentLimits.SupplyCurrentLimit = IntakeConstants.DRUM_CONTINUOUS_CURRENT_LIMIT;
    drumCurrentLimits.SupplyCurrentThreshold = IntakeConstants.DRUM_PEAK_CURRENT_LIMIT;
    drumCurrentLimits.SupplyTimeThreshold = IntakeConstants.DRUM_PEAK_CURRENT_DURATION;
    drumCurrentLimits.SupplyCurrentLimitEnable = true;

    drumConfig.CurrentLimits = drumCurrentLimits;

    drumConfig.MotorOutput.Inverted =
        IntakeConstants.DRUM_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    drumMotor.getConfigurator().apply(drumConfig);
    
  }
}
