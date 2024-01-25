package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterIOTalonFX {

  private VoltageOut voltageRequest;
  private TorqueCurrentFOC currentRequest;
  private PositionVoltage positionRequest;

  private TalonFX shootMotorLeft;
  private TalonFX shootMotorRight;
  private TalonFX angleMotor;
  private TalonFX drumMotor;
  private CANcoder angleEncoder;
  private final DigitalInput shooterSensor;

  public ShooterIOTalonFX() {
    configMotors(
        ShooterConstants.LEFT_SHOOTER_MOTOR_ID,
        RIGHT__SHOOTER_MOTOR_ID,
        ANGLE_MOTOR_ID,
        DRUM_MOTOR_ID);
  }
}
