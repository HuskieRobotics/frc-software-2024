package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.team3015.subsystem.FaultReporter;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.Alert;
import frc.lib.team6328.util.Alert.AlertType;
import frc.lib.team6328.util.TunableNumber;

public class ClimberIOTalonFX implements ClimberIO {
  private TalonFX climberMotor;

  private VoltageOut climberVoltageRequest;

  private final TunableNumber kP = new TunableNumber("Climber/KP", ClimberConstants.KP);
  private final TunableNumber kI = new TunableNumber("Climber/KI", ClimberConstants.KI);
  private final TunableNumber kD = new TunableNumber("Climber/KD", ClimberConstants.KD);

  private final TunableNumber kS = new TunableNumber("Climber/KS", ClimberConstants.KS);
  private final TunableNumber kV = new TunableNumber("Climber/KV", ClimberConstants.KV);
  private final TunableNumber kA = new TunableNumber("Climber/KA", ClimberConstants.KA);
  private final TunableNumber kVExpo =
      new TunableNumber("Climber/KVExpo", ClimberConstants.KV_EXPO);
  private final TunableNumber kAExpo =
      new TunableNumber("Climber/KAExpo", ClimberConstants.KA_EXPO);
  private final TunableNumber kG = new TunableNumber("Climber/KG", ClimberConstants.KG);

  private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

  private StatusSignal<Double> climberStatorCurrentStatusSignal;
  private StatusSignal<Double> climberSupplyCurrentStatusSignal;
  private StatusSignal<Double> climberVoltageStatusSignal;
  private StatusSignal<Double> climberPositionStatusSignal;
  private StatusSignal<Double> climberTemperatureStatusSignal;

  public ClimberIOTalonFX() {
    climberMotor =
        new TalonFX(ClimberConstants.CLIMBER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());

    climberVoltageRequest = new VoltageOut(0);

    climberStatorCurrentStatusSignal = climberMotor.getStatorCurrent();
    climberSupplyCurrentStatusSignal = climberMotor.getSupplyCurrent();
    climberVoltageStatusSignal = climberMotor.getMotorVoltage();
    climberPositionStatusSignal = climberMotor.getPosition();
    climberTemperatureStatusSignal = climberMotor.getDeviceTemp();

    configureClimberMotor(climberMotor);

    climberMotor.setPosition(0.0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        climberStatorCurrentStatusSignal,
        climberSupplyCurrentStatusSignal,
        climberVoltageStatusSignal,
        climberPositionStatusSignal,
        climberTemperatureStatusSignal);

    inputs.climberMotorStatorCurrentAmps = climberStatorCurrentStatusSignal.getValueAsDouble();
    inputs.climberMotorSupplyCurrentAmps = climberSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.climberMotorVoltage = climberVoltageStatusSignal.getValueAsDouble();
    inputs.climberMotorPositionRotations = climberPositionStatusSignal.getValueAsDouble();
    inputs.climberMotorTemperatureCelsius = climberTemperatureStatusSignal.getValueAsDouble();

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

      this.climberMotor.getConfigurator().refresh(config);
      config.Slot0.kP = kP.get();
      config.Slot0.kI = kI.get();
      config.Slot0.kD = kD.get();
      config.Slot0.kS = kS.get();
      config.Slot0.kV = kV.get();
      config.Slot0.kA = kA.get();
      motionMagicConfigs.MotionMagicExpo_kV = kVExpo.get();
      motionMagicConfigs.MotionMagicExpo_kA = kAExpo.get();
      config.Slot0.kG = kG.get();
    }
  }

  @Override
  public void setClimberVoltage(double voltage) {
    climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
  }

  @Override
  public void zeroPosition() {
    climberMotor.setPosition(0.0);
  }

  private void configureClimberMotor(TalonFX motor) {
    TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs climberMotorCurrentLimits = new CurrentLimitsConfigs();
    climberMotorCurrentLimits.SupplyCurrentLimit =
        ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyCurrentThreshold = ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
    climberMotorCurrentLimits.SupplyTimeThreshold = ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
    climberMotorCurrentLimits.SupplyCurrentLimitEnable = true;
    climberMotorConfig.CurrentLimits = climberMotorCurrentLimits;

    climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    climberMotorConfig.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

    climberMotorConfig.MotorOutput.Inverted =
        ClimberConstants.CLIMBER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = motor.getConfigurator().apply(climberMotorConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      configAlert.set(true);
      configAlert.setText(status.toString());
    }

    FaultReporter.getInstance()
        .registerHardware(ClimberConstants.SUBSYSTEM_NAME, "ClimberMotor", motor);
  }
}
