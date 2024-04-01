package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
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
  private VoltageOut kickerVoltageRequest;

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

  private StatusSignal<Double> rollerVoltageStatusSignal;
  private StatusSignal<Double> kickerVoltageStatusSignal;

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
    kickerVoltageRequest = new VoltageOut(0);

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

    rollerVoltageStatusSignal = rollerMotor.getMotorVoltage();
    kickerVoltageStatusSignal = kickerMotor.getMotorVoltage();

    this.rollerMotorSim =
        new VelocitySystemSim(
            rollerMotor,
            IntakeConstants.ROLLER_MOTOR_INVERTED,
            0.02,
            0.001,
            ROLLERS_SENSOR_TO_MECHANISM_RATIO);
    this.kickerMotorSim =
        new VelocitySystemSim(
            kickerMotor,
            IntakeConstants.KICKER_MOTOR_INVERTED,
            1.0,
            0.3,
            KICKER_SENSOR_TO_MECHANISM_RATIO);
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
        kickerTemperatureStatusSignal,
        rollerVoltageStatusSignal,
        kickerVoltageStatusSignal);

    inputs.isRollerIRBlocked = !rollerIRSensor.get();
    inputs.isKickerIRBlocked = !kickerIRSensor.get();
    inputs.isShooterIRBlocked = !shooterIRSensor.get();

    inputs.rollerStatorCurrentAmps = rollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.kickerStatorCurrentAmps = kickerStatorCurrentStatusSignal.getValueAsDouble();

    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.kickerSupplyCurrentAmps = kickerSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.rollerVelocityRPS = rollerVelocityStatusSignal.getValueAsDouble();
    inputs.kickerVelocityRPS = kickerVelocityStatusSignal.getValueAsDouble();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.rollerReferenceVelocityRPS = rollerMotor.getClosedLoopReference().getValueAsDouble();
    inputs.kickerReferenceVelocityRPS = kickerMotor.getClosedLoopReference().getValueAsDouble();

    inputs.rollerTempCelsius = rollerTemperatureStatusSignal.getValueAsDouble();
    inputs.kickerTempCelsius = kickerTemperatureStatusSignal.getValueAsDouble();

    inputs.rollerVoltage = rollerVoltageStatusSignal.getValueAsDouble();
    inputs.kickerVoltage = kickerVoltageStatusSignal.getValueAsDouble();

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

  @Override
  public void setKickerVoltage(double voltage) {
    kickerMotor.setControl(kickerVoltageRequest.withOutput(voltage));
  }

  private void configureIntakeRollerMotor(TalonFX rollerMotor) {
    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs rollerTorqueCurrentConfigs = new TorqueCurrentConfigs();

    rollerTorqueCurrentConfigs.PeakForwardTorqueCurrent =
        IntakeConstants.ROLLERS_CONTINUOUS_STATOR_CURRENT_LIMIT;
    rollerTorqueCurrentConfigs.PeakReverseTorqueCurrent =
        -IntakeConstants.ROLLERS_CONTINUOUS_STATOR_CURRENT_LIMIT;

    // FIXME: need to limit intake current but not so much that we can't intake a note; perhaps
    // switch to voltage control where we can use more sophisticated current limiting?
    // rollerConfig.TorqueCurrent = rollerTorqueCurrentConfigs;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
    for (int i = 0; i < 5; ++i) {
      status = rollerMotor.getConfigurator().apply(rollerConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance().registerHardware("INTAKE", "IntakeRoller", rollerMotor);
  }

  private void configureIntakeKickerMotor(TalonFX kickerMotor) {
    TalonFXConfiguration kickerConfig = new TalonFXConfiguration();
    TorqueCurrentConfigs kickerTorqueCurrentConfigs = new TorqueCurrentConfigs();

    kickerTorqueCurrentConfigs.PeakForwardTorqueCurrent =
        IntakeConstants.KICKER_PEAK_SUPPLY_CURRENT_LIMIT;
    kickerTorqueCurrentConfigs.PeakReverseTorqueCurrent =
        -IntakeConstants.KICKER_PEAK_SUPPLY_CURRENT_LIMIT;

    kickerConfig.TorqueCurrent = kickerTorqueCurrentConfigs;

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.Slot0.kP = kickerMotorKP.get();
    kickerConfig.Slot0.kI = kickerMotorKI.get();
    kickerConfig.Slot0.kD = kickerMotorKD.get();
    kickerConfig.Slot0.kS = kickerMotorKS.get();

    kickerConfig.MotorOutput.Inverted =
        IntakeConstants.KICKER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    kickerConfig.Feedback.SensorToMechanismRatio = IntakeConstants.KICKER_SENSOR_TO_MECHANISM_RATIO;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = kickerMotor.getConfigurator().apply(kickerConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance().registerHardware("INTAKE", "IntakeKicker", kickerMotor);
  }
}
