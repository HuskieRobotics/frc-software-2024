package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ClimberIOTalonFX implements ClimberIO {
    private TalonFX climberMotor;
    private final String SUBSYSTEM_NAME = "Climber";

    private VoltageOut climberVoltageRequest;

    private Alert configAlert =
      new Alert("Failed to apply configuration for subsystem.", AlertType.ERROR);

    private StatusSignal<Double> climberStatorCurrentStatusSignal;
    private StatusSignal<Double> climberSupplyCurrentStatusSignal;
    private StatusSignal<Double> climberVoltageStatusSignal;
    private StatusSignal<Double> climberTemperatureStatusSignal;

    public ClimberIOTalonFX(TalonFX climberMotor) {
        climberMotor = new TalonFX(
            ClimberConstants.CLIMBER_MOTOR_ID, RobotConfig.getInstance().getCANBusName());
        
        climberVoltageRequest = new VoltageOut(0);

        climberStatorCurrentStatusSignal = climberMotor.getStatorCurrent();
        climberSupplyCurrentStatusSignal = climberMotor.getSupplyCurrent();
        climberVoltageStatusSignal = climberMotor.getMotorVoltage();
        climberTemperatureStatusSignal = climberMotor.getDeviceTemp();

        configureClimberMotor(climberMotor);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            climberStatorCurrentStatusSignal, 
            climberSupplyCurrentStatusSignal,
            climberVoltageStatusSignal, 
            climberTemperatureStatusSignal
        );
        
        inputs.climberMotorStatorCurrentAmps = climberStatorCurrentStatusSignal.getValueAsDouble();
        inputs.climberMotorSupplyCurrentAmps = climberSupplyCurrentStatusSignal.getValueAsDouble();
        inputs.climberMotorVoltage = climberVoltageStatusSignal.getValueAsDouble();
        inputs.climberMotorTemperatureCelsius = climberTemperatureStatusSignal.getValueAsDouble();
    }

    @Override
    public void setClimberVoltage(double voltage) {
        climberMotor.setControl(climberVoltageRequest.withOutput(voltage));
    }

    private void configureClimberMotor(TalonFX motor) {
        TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs climberMotorCurrentLimits = new CurrentLimitsConfigs();
        climberMotorCurrentLimits.SupplyCurrentLimit =
            ClimberConstants.CLIMBER_CONTINUOUS_CURRENT_LIMIT;
        climberMotorCurrentLimits.SupplyCurrentThreshold =
            ClimberConstants.CLIMBER_PEAK_CURRENT_LIMIT;
        climberMotorCurrentLimits.SupplyTimeThreshold =
            ClimberConstants.CLIMBER_PEAK_CURRENT_DURATION;
        climberMotorCurrentLimits.SupplyCurrentLimitEnable = true;
        climberMotorConfig.CurrentLimits = climberMotorCurrentLimits;

        climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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

        FaultReporter.getInstance().registerHardware(SUBSYSTEM_NAME, "ClimberMotor", motor);
    }


}
