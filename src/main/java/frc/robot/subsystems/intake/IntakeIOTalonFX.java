package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

  // tunable numbers for roller and drum pid
  private final TunableNumber rightRollerMotorKP =
      new TunableNumber("Intake/rightRollerMotorKP", IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_KP);
  private final TunableNumber rightRollerMotorKI =
      new TunableNumber("Intake/rightRollerMotorKI", IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_KI);
  private final TunableNumber rightRollerMotorKD =
      new TunableNumber("Intake/rightRollerMotorKD", IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_KD);

  private final TunableNumber leftRollerMotorKP =
      new TunableNumber("Intake/leftRollerMotorKP", IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_KP);
  private final TunableNumber leftRollerMotorKI = 
      new TunableNumber("Intake/leftRollerMotorKI", IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_KI);
  private final TunableNumber leftRollerMotorKD = 
      new TunableNumber("Intake/leftRollerMotorKD", IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_KD);

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

    inputs.rightRollerAppliedPercentage = rightRollerMotor.getDutyCycle().getValueAsDouble();
    inputs.leftRollerAppliedPercentage = leftRollerMotor.getDutyCycle().getValueAsDouble();
    inputs.drumAppliedPercentage = drumMotor.getDutyCycle().getValueAsDouble();

    inputs.rightRollerStatorCurrentAmps =
        new double[] {rightRollerMotor.getStatorCurrent().getValueAsDouble()};
    inputs.leftRollerStatorCurrentAmps =
        new double[] {leftRollerMotor.getStatorCurrent().getValueAsDouble()};
    inputs.drumStatorCurrentAmps = new double[] {drumMotor.getStatorCurrent().getValueAsDouble()};
  }

  @Override
  public void setRightRollerPower(double percentage) {
    rightRollerMotor.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setLeftRollerPower(double percentage) {
    leftRollerMotor.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setDrumPower(double percentage) {
    drumMotor.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setRightRollerCurrent(double current) {
    rightRollerMotor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void setLeftRollerCurrent(double current) {
    leftRollerMotor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void setDrumCurrent(double current) {
    drumMotor.setControl(new TorqueCurrentFOC(current));
  }

  private void configureIntakeMotors() {
    rightRollerMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_ID);
    leftRollerMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_ID);
    drumMotor = new TalonFX(IntakeConstants.INTAKE_DRUM_MOTOR_ID);

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
