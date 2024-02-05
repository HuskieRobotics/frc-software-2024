package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rightRollerMotor;
  private TalonFX leftRollerMotor;
  private TalonFX drumMotor;
  private TalonFX kickerMotor;
  private final DigitalInput rightRollerIRSensor;
  private final DigitalInput leftRollerIRSensor;
  private final DigitalInput drumIRSensor;
  private final DigitalInput kickerIRSensor;

  private VelocityTorqueCurrentFOC rightRollerVelocityRequest;
  private VelocityTorqueCurrentFOC leftRollerVelocityRequest;
  private VelocityTorqueCurrentFOC drumVelocityRequest;
  private VelocityTorqueCurrentFOC kickerVelocityRequest;

  private StatusSignal<Double> rightRollerVelocityStatusSignal;
  private StatusSignal<Double> leftRollerVelocityStatusSignal;
  private StatusSignal<Double> drumVelocityStatusSignal;
  private StatusSignal<Double> kickerVelocityStatusSignal;
  // status signals for stator current
  private StatusSignal<Double> rightRollerStatorCurrentStatusSignal;
  private StatusSignal<Double> leftRollerStatorCurrentStatusSignal;
  private StatusSignal<Double> drumStatorCurrentStatusSignal;
  private StatusSignal<Double> kickerStatorCurrentStatusSignal;

  private StatusSignal<Double> rightRollerSupplyCurrentStatusSignal;
  private StatusSignal<Double> leftRollerSupplyCurrentStatusSignal;
  private StatusSignal<Double> drumSupplyCurrentStatusSignal;
  private StatusSignal<Double> kickerSupplyCurrentStatusSignal;

  // tunable numbers for roller and drum pid
  private final TunableNumber rollerMotorsKP =
      new TunableNumber("Intake/rollerMotorsKP", IntakeConstants.INTAKE_ROLLER_MOTORS_KP);

  private final TunableNumber rollerMotorsKI =
      new TunableNumber("Intake/rollerMotorsKI", IntakeConstants.INTAKE_ROLLER_MOTORS_KI);

  private final TunableNumber rollerMotorsKD =
      new TunableNumber("Intake/rollerMotorsKD", IntakeConstants.INTAKE_ROLLER_MOTORS_KD);

  private final TunableNumber rollerMotorsKS =
      new TunableNumber("Intake/rollerMotorsKS", IntakeConstants.INTAKE_ROLLER_MOTORS_KS);

  private final TunableNumber drumMotorKP =
      new TunableNumber("Intake/drumMotorKP", IntakeConstants.INTAKE_DRUM_MOTOR_KP);
  private final TunableNumber drumMotorKI =
      new TunableNumber("Intake/drumMotorKI", IntakeConstants.INTAKE_DRUM_MOTOR_KI);
  private final TunableNumber drumMotorKD =
      new TunableNumber("Intake/drumMotorKD", IntakeConstants.INTAKE_DRUM_MOTOR_KD);

  private final TunableNumber drumMotorKS =
      new TunableNumber("Intake/drumMotorKS", IntakeConstants.INTAKE_DRUM_MOTOR_KS);

  private final TunableNumber kickerMotorKP =
      new TunableNumber("Intake/kickerMotorKP", IntakeConstants.INTAKE_KICKER_MOTOR_KP);
    
  private final TunableNumber kickerMotorKI =
      new TunableNumber("Intake/kickerMotorKI", IntakeConstants.INTAKE_KICKER_MOTOR_KI);
  
  private final TunableNumber kickerMotorKD =
      new TunableNumber("Intake/kickerMotorKD", IntakeConstants.INTAKE_KICKER_MOTOR_KD);
  
  private final TunableNumber kickerMotorKS =
      new TunableNumber("Intake/kickerMotorKS", IntakeConstants.INTAKE_KICKER_MOTOR_KS);

  public IntakeIOTalonFX() {
    rightRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_RIGHT_ROLLER_IR_SENSOR_ID);
    leftRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_LEFT_ROLLER_IR_SENSOR_ID);
    drumIRSensor = new DigitalInput(IntakeConstants.INTAKE_DRUM_IR_SENSOR_ID);
    kickerIRSensor = new DigitalInput(IntakeConstants.INTAKE_KICKER_IR_SENSOR_ID);

    rightRollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    leftRollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    drumVelocityRequest = new VelocityTorqueCurrentFOC(0);
    kickerVelocityRequest = new VelocityTorqueCurrentFOC(0);

    rightRollerVelocityStatusSignal = rightRollerMotor.getVelocity();
    leftRollerVelocityStatusSignal = leftRollerMotor.getVelocity();
    drumVelocityStatusSignal = drumMotor.getVelocity();
    kickerVelocityStatusSignal = kickerMotor.getVelocity();

    rightRollerStatorCurrentStatusSignal = rightRollerMotor.getStatorCurrent();
    leftRollerStatorCurrentStatusSignal = leftRollerMotor.getStatorCurrent();
    drumStatorCurrentStatusSignal = drumMotor.getStatorCurrent();
    kickerStatorCurrentStatusSignal = kickerMotor.getStatorCurrent();

    rightRollerSupplyCurrentStatusSignal = rightRollerMotor.getSupplyCurrent();
    leftRollerSupplyCurrentStatusSignal = leftRollerMotor.getSupplyCurrent();
    drumSupplyCurrentStatusSignal = drumMotor.getSupplyCurrent();
    kickerSupplyCurrentStatusSignal = kickerMotor.getSupplyCurrent();

    rightRollerMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    leftRollerMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    drumMotor = new TalonFX(IntakeConstants.INTAKE_DRUM_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    kickerMotor = new TalonFX(IntakeConstants.INTAKE_KICKER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    configureIntakeRollerMotor(rightRollerMotor, true);
    configureIntakeRollerMotor(leftRollerMotor, false);
    configureIntakeDrumMotor(drumMotor);
    configureIntakeKickerMotor(kickerMotor);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rightRollerVelocityStatusSignal,
        leftRollerVelocityStatusSignal,
        drumVelocityStatusSignal,
        rightRollerStatorCurrentStatusSignal,
        leftRollerStatorCurrentStatusSignal,
        drumStatorCurrentStatusSignal,
        rightRollerSupplyCurrentStatusSignal,
        leftRollerSupplyCurrentStatusSignal,
        drumSupplyCurrentStatusSignal,
        kickerSupplyCurrentStatusSignal,
        kickerStatorCurrentStatusSignal,
        kickerVelocityStatusSignal);

    inputs.isRightRollerIRBlocked = rightRollerIRSensor.get();
    inputs.isLeftRollerIRBlocked = leftRollerIRSensor.get();
    inputs.isDrumIRBlocked = drumIRSensor.get();
    inputs.isKickerIRBlocked = kickerIRSensor.get();

    inputs.rightRollerStatorCurrentAmps = rightRollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.leftRollerStatorCurrentAmps = leftRollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.drumStatorCurrentAmps = drumStatorCurrentStatusSignal.getValueAsDouble();
    inputs.kickerStatorCurrentAmps = kickerStatorCurrentStatusSignal.getValueAsDouble();

    inputs.rightRollerSupplyCurrentAmps = rightRollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.leftRollerSupplyCurrentAmps = leftRollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.drumSupplyCurrentAmps = drumSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.kickerSupplyCurrentAmps = kickerSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.rightRollerVelocityRotationsPerSecond =
        rightRollerVelocityStatusSignal.getValueAsDouble();
    inputs.leftRollerVelocityRotationsPerSecond = leftRollerVelocityStatusSignal.getValueAsDouble();
    inputs.drumVelocityRotationsPerSecond = drumVelocityStatusSignal.getValueAsDouble();
    inputs.kickerVelocityRotationsPerSecond = kickerVelocityStatusSignal.getValueAsDouble();

    // what method would be used to get the value of the reference velocity as a status signal?
    // could not find any methods that returned a status signal for the reference velocity
    inputs.rightRollerReferenceVelocityRPS = rightRollerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.leftRollerReferenceVelocityRPS = leftRollerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.drumReferenceVelocityRPS = drumMotor.getClosedLoopReference().getValueAsDouble();
    inputs.kickerReferenceVelocityRPS = kickerMotor.getClosedLoopReference().getValueAsDouble();

    if (rollerMotorsKP.hasChanged()
      || rollerMotorsKI.hasChanged()
      || rollerMotorsKD.hasChanged()
      || rollerMotorsKS.hasChanged() 
      || drumMotorKP.hasChanged()
      || drumMotorKI.hasChanged()
      || drumMotorKD.hasChanged()
      || drumMotorKS.hasChanged()
      || kickerMotorKP.hasChanged()
      || kickerMotorKI.hasChanged()
      || kickerMotorKD.hasChanged()
      || kickerMotorKS.hasChanged()){
        
       for (TalonFX motor : new TalonFX[] {rightRollerMotor, leftRollerMotor, drumMotor, kickerMotor}) {
          Slot0Configs slot0Configs = new Slot0Configs();
          motor.getConfigurator().refresh(slot0Configs);
          slot0Configs.kP = rollerMotorsKP.get();
          slot0Configs.kI = rollerMotorsKI.get();
          slot0Configs.kD = rollerMotorsKD.get();
          slot0Configs.kS = rollerMotorsKS.get();

          motor.getConfigurator().apply(slot0Configs);
       }

      }
  }

  @Override
  public void setRightRollerVelocity(double rps) {
    rightRollerMotor.setControl(rightRollerVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setLeftRollerVelocity(double rps) {
    leftRollerMotor.setControl(leftRollerVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setDrumVelocity(double rps) {
    drumMotor.setControl(drumVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setKickerVelocity(double rps) {
    kickerMotor.setControl(kickerVelocityRequest.withVelocity(rps));
  }

  private void configureIntakeRollerMotor(TalonFX rollerMotor, boolean isRight) {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();

    rollerCurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLERS_CONTINUOUS_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyCurrentThreshold = IntakeConstants.ROLLERS_PEAK_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyTimeThreshold = IntakeConstants.ROLLERS_PEAK_CURRENT_DURATION;
    rollerCurrentLimits.SupplyCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimits;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rollerConfig.Slot0.kP = rollerMotorsKP.get();
    rollerConfig.Slot0.kI = rollerMotorsKI.get();
    rollerConfig.Slot0.kD = rollerMotorsKD.get();
    rollerConfig.Slot0.kS = rollerMotorsKS.get();

    if (isRight) {
      rollerConfig.MotorOutput.Inverted =
          IntakeConstants.RIGHT_ROLLER_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
    } else {
      rollerConfig.MotorOutput.Inverted =
          IntakeConstants.LEFT_ROLLER_MOTOR_INVERTED
              ? InvertedValue.Clockwise_Positive
              : InvertedValue.CounterClockwise_Positive;
    }

    rollerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.ROLLERS_SENSOR_TO_MECHANISM_RATIO;

    rollerMotor.getConfigurator().apply(rollerConfig);
  }

  private void configureIntakeDrumMotor(TalonFX drumMotor) {
    TalonFXConfiguration drumConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs drumCurrentLimits = new CurrentLimitsConfigs();

    drumCurrentLimits.SupplyCurrentLimit = IntakeConstants.DRUM_CONTINUOUS_CURRENT_LIMIT;
    drumCurrentLimits.SupplyCurrentThreshold = IntakeConstants.DRUM_PEAK_CURRENT_LIMIT;
    drumCurrentLimits.SupplyTimeThreshold = IntakeConstants.DRUM_PEAK_CURRENT_DURATION;
    drumCurrentLimits.SupplyCurrentLimitEnable = true;

    drumConfig.CurrentLimits = drumCurrentLimits;

    drumConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    drumConfig.Slot0.kP = drumMotorKP.get();
    drumConfig.Slot0.kI = drumMotorKI.get();
    drumConfig.Slot0.kD = drumMotorKD.get();
    drumConfig.Slot0.kS = drumMotorKS.get();

    drumConfig.MotorOutput.Inverted =
        IntakeConstants.DRUM_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    drumMotor.getConfigurator().apply(drumConfig);
  }

  private void configureIntakeKickerMotor(TalonFX kickerMotor) {
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs kickerCurrentLimits = new CurrentLimitsConfigs();

    kickerCurrentLimits.SupplyCurrentLimit = IntakeConstants.KICKER_CONTINUOUS_CURRENT_LIMIT;
    kickerCurrentLimits.SupplyCurrentThreshold = IntakeConstants.DRUM_PEAK_CURRENT_LIMIT;
    kickerCurrentLimits.SupplyTimeThreshold = IntakeConstants.DRUM_PEAK_CURRENT_DURATION;
    kickerCurrentLimits.SupplyCurrentLimitEnable = true;

    kickerConfig.CurrentLimits = kickerCurrentLimits;

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.Slot0.kP = kickerMotorKP.get();
    kickerConfig.Slot0.kI = kickerMotorKI.get();
    kickerConfig.Slot0.kD = kickerMotorKD.get();
    kickerConfig.Slot0.kS = kickerMotorKS.get();

    kickerConfig.MotorOutput.Inverted =
        IntakeConstants.KICKER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    
    kickerMotor.getConfigurator().apply(kickerConfig);
  }
}
