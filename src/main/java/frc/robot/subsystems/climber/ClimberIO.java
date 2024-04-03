package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    
    @AutoLog
    public static class ClimberIOInputs {
        double climberMotorStatorCurrentAmps = 0.0;
        double climberMotorSupplyCurrentAmps = 0.0;
        double climberMotorVoltage = 0.0;
        double climberMotorTemperatureCelsius = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setClimberVoltage(double voltage) {}
}
