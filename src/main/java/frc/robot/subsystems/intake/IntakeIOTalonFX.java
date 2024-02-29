package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.util.VelocitySystemSim;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rollerMotor;
  private TalonFX kickerMotor;
  private final DigitalInput rollerIRSensor;
  private final DigitalInput kickerIRSensor;
  private final DigitalInput shooterIRSensor;

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  private VelocityTorqueCurrentFOC rollerVelocityRequest;
  private VelocityTorqueCurrentFOC kickerVelocityRequest;

  private StatusSignal<Double> rollerVelocityStatusSignal;
  private StatusSignal<Double> kickerVelocityStatusSignal;

  private StatusSignal<Double> rollerStatorCurrentStatusSignal;
  private StatusSignal<Double> kickerStatorCurrentStatusSignal;

  private StatusSignal<Double> rollerSupplyCurrentStatusSignal;
  private StatusSignal<Double> kickerSupplyCurrentStatusSignal;

  private StatusSignal<Double> rollerReferenceVelocityStatusSignal;
  private StatusSignal<Double> kickerReferenceVelocityStatusSignal;

  private StatusSignal<Double> rollerTemperatureStatusSignal;
  private StatusSignal<Double> kickerTemperatureStatusSignal;

  // simulation related
  private VelocitySystemSim rollerMotorSim;
  private VelocitySystemSim kickerMotorSim;

  // tunable numbers for roller and drum pid
  private final TunableNumber rollerMotorsKP =
      new TunableNumber("Intake/rollerMotorsKP", IntakeConstants.INTAKE_ROLLER_MOTORS_KP);

  private final TunableNumber rollerMotorsKI =
      new TunableNumber("Intake/rollerMotorsKI", IntakeConstants.INTAKE_ROLLER_MOTORS_KI);

  private final TunableNumber rollerMotorsKD =
      new TunableNumber("Intake/rollerMotorsKD", IntakeConstants.INTAKE_ROLLER_MOTORS_KD);

  private final TunableNumber rollerMotorsKS =
      new TunableNumber("Intake/rollerMotorsKS", IntakeConstants.INTAKE_ROLLER_MOTORS_KS);

  private final TunableNumber kickerMotorKP =
      new TunableNumber("Intake/kickerMotorKP", IntakeConstants.INTAKE_KICKER_MOTOR_KP);

  private final TunableNumber kickerMotorKI =
      new TunableNumber("Intake/kickerMotorKI", IntakeConstants.INTAKE_KICKER_MOTOR_KI);

  private final TunableNumber kickerMotorKD =
      new TunableNumber("Intake/kickerMotorKD", IntakeConstants.INTAKE_KICKER_MOTOR_KD);

  private final TunableNumber kickerMotorKS =
      new TunableNumber("Intake/kickerMotorKS", IntakeConstants.INTAKE_KICKER_MOTOR_KS);

  public IntakeIOTalonFX() {
    rollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_ROLLER_IR_SENSOR_ID);
    kickerIRSensor = new DigitalInput(IntakeConstants.INTAKE_KICKER_IR_SENSOR_ID);
    shooterIRSensor = new DigitalInput(IntakeConstants.INTAKE_SHOOTER_IR_SENSOR_ID);

    // torque current FOC control mode is not support in simulation yet

    rollerMotor =
        new TalonFX(
            IntakeConstants.INTAKE_ROLLER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
    kickerMotor =
        new TalonFX(
            IntakeConstants.INTAKE_KICKER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    configureIntakeRollerMotor(rollerMotor);
    configureIntakeKickerMotor(kickerMotor);

    rollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    kickerVelocityRequest = new VelocityTorqueCurrentFOC(0);

    rollerVelocityStatusSignal = rollerMotor.getVelocity();
    kickerVelocityStatusSignal = kickerMotor.getVelocity();

    rollerStatorCurrentStatusSignal = rollerMotor.getStatorCurrent();
    kickerStatorCurrentStatusSignal = kickerMotor.getStatorCurrent();

    rollerSupplyCurrentStatusSignal = rollerMotor.getSupplyCurrent();
    kickerSupplyCurrentStatusSignal = kickerMotor.getSupplyCurrent();

    rollerReferenceVelocityStatusSignal = rollerMotor.getClosedLoopReference();
    kickerReferenceVelocityStatusSignal = kickerMotor.getClosedLoopReference();

    rollerTemperatureStatusSignal = rollerMotor.getDeviceTemp();
    kickerTemperatureStatusSignal = kickerMotor.getDeviceTemp();

    this.rollerMotorSim =
        new VelocitySystemSim(
            rollerMotor,
            IntakeConstants.ROLLER_MOTOR_INVERTED,
            0.02,
            0.001,
            ROLLERS_SENSOR_TO_MECHANISM_RATIO);
    this.kickerMotorSim =
        new VelocitySystemSim(kickerMotor, IntakeConstants.KICKER_MOTOR_INVERTED, 1.0, 0.3, 1.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    this.rollerMotorSim.updateSim();
    this.kickerMotorSim.updateSim();

    BaseStatusSignal.refreshAll(
        rollerVelocityStatusSignal,
        rollerStatorCurrentStatusSignal,
        rollerSupplyCurrentStatusSignal,
        rollerReferenceVelocityStatusSignal,
        kickerSupplyCurrentStatusSignal,
        kickerStatorCurrentStatusSignal,
        kickerVelocityStatusSignal,
        kickerReferenceVelocityStatusSignal,
        rollerTemperatureStatusSignal,
        kickerTemperatureStatusSignal);

    inputs.isRollerIRBlocked = !rollerIRSensor.get();
    inputs.isKickerIRBlocked = !kickerIRSensor.get();
    inputs.isShooterIRBlocked = !shooterIRSensor.get();

    inputs.rollerStatorCurrentAmps = rollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.kickerStatorCurrentAmps = kickerStatorCurrentStatusSignal.getValueAsDouble();

    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.kickerSupplyCurrentAmps = kickerSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.rollerVelocityRPS = rollerVelocityStatusSignal.getValueAsDouble();
    inputs.kickerVelocityRPS = kickerVelocityStatusSignal.getValueAsDouble();

    inputs.rollerReferenceVelocityRPS = rollerReferenceVelocityStatusSignal.getValueAsDouble();
    inputs.kickerReferenceVelocityRPS = kickerReferenceVelocityStatusSignal.getValueAsDouble();

    inputs.rollerTempCelcius = rollerTemperatureStatusSignal.getValueAsDouble();
    inputs.kickerTempCelcius = kickerTemperatureStatusSignal.getValueAsDouble();

    if (rollerMotorsKP.hasChanged()
        || rollerMotorsKI.hasChanged()
        || rollerMotorsKD.hasChanged()
        || rollerMotorsKS.hasChanged()) {

      Slot0Configs slot0Configs = new Slot0Configs();
      rollerMotor.getConfigurator().refresh(slot0Configs);
      slot0Configs.kP = rollerMotorsKP.get();
      slot0Configs.kI = rollerMotorsKI.get();
      slot0Configs.kD = rollerMotorsKD.get();
      slot0Configs.kS = rollerMotorsKS.get();

      rollerMotor.getConfigurator().apply(slot0Configs);
    }

    if (kickerMotorKP.hasChanged()
        || kickerMotorKI.hasChanged()
        || kickerMotorKD.hasChanged()
        || kickerMotorKS.hasChanged()) {
      Slot0Configs slot0Configs = new Slot0Configs();
      kickerMotor.getConfigurator().refresh(slot0Configs);
      slot0Configs.kP = rollerMotorsKP.get();
      slot0Configs.kI = rollerMotorsKI.get();
      slot0Configs.kD = rollerMotorsKD.get();
      slot0Configs.kS = rollerMotorsKS.get();

      kickerMotor.getConfigurator().apply(slot0Configs);
    }
  }

  @Override
  public void setRollerVelocity(double rps) {
    rollerMotor.setControl(rollerVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setKickerVelocity(double rps) {
    kickerMotor.setControl(kickerVelocityRequest.withVelocity(rps));
  }

  private void configureIntakeRollerMotor(TalonFX rollerMotor) {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();

    rollerCurrentLimits.SupplyCurrentLimit =
        IntakeConstants.ROLLERS_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyCurrentThreshold = IntakeConstants.ROLLERS_PEAK_SUPPLY_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyTimeThreshold = IntakeConstants.ROLLERS_PEAK_SUPPLY_CURRENT_DURATION;

    rollerCurrentLimits.StatorCurrentLimit =
        IntakeConstants.ROLLERS_CONTINUOUS_STATOR_CURRENT_LIMIT;

    rollerCurrentLimits.SupplyCurrentLimitEnable = true;
    rollerCurrentLimits.StatorCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimits;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    rollerConfig.Slot0.kP = rollerMotorsKP.get();
    rollerConfig.Slot0.kI = rollerMotorsKI.get();
    rollerConfig.Slot0.kD = rollerMotorsKD.get();
    rollerConfig.Slot0.kS = rollerMotorsKS.get();

    rollerConfig.MotorOutput.Inverted =
        IntakeConstants.ROLLER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    rollerConfig.Feedback.SensorToMechanismRatio =
        IntakeConstants.ROLLERS_SENSOR_TO_MECHANISM_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = rollerMotor.getConfigurator().apply(rollerConfig);

    if (status != StatusCode.OK) {
      configAlert.set(false);
    }

    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance().registerHardware("INTAKE", "IntakeRoller", rollerMotor);
  }

  private void configureIntakeKickerMotor(TalonFX kickerMotor) {
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs kickerCurrentLimits = new CurrentLimitsConfigs();

    kickerCurrentLimits.SupplyCurrentLimit = IntakeConstants.KICKER_CONTINUOUS_SUPPLY_CURRENT_LIMIT;
    kickerCurrentLimits.SupplyCurrentThreshold = IntakeConstants.KICKER_PEAK_SUPPLY_CURRENT_LIMIT;
    kickerCurrentLimits.SupplyTimeThreshold = IntakeConstants.KICKER_PEAK_SUPPLY_CURRENT_DURATION;

    kickerCurrentLimits.StatorCurrentLimit = IntakeConstants.KICKER_CONTINUOUS_STATOR_CURRENT_LIMIT;

    kickerCurrentLimits.StatorCurrentLimitEnable = true;
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

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    status = rollerMotor.getConfigurator().apply(kickerConfig);

    if (status != StatusCode.OK) {
      configAlert.set(false);
    }

    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance().registerHardware("INTAKE", "IntakeKicker", kickerMotor);
  }
}
