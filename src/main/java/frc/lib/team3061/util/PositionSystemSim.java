package frc.lib.team3061.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.Constants;

public class PositionSystemSim {

  private TalonFX motor;
  private CANcoder encoder;
  private TalonFXSimState motorSimState;
  private CANcoderSimState encoderSimState;
  private LinearSystemSim<N2, N1, N1> systemSim;
  private double gearRatio;
  private double prevPosition = 0.0;

  public PositionSystemSim(
      TalonFX motor,
      CANcoder encoder,
      boolean motorInverted,
      double kV,
      double kA,
      double gearRatio) {
    this.motor = motor;
    this.encoder = encoder;
    this.gearRatio = gearRatio;

    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.motorSimState = this.motor.getSimState();
    this.motorSimState.Orientation =
        motorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    this.encoderSimState = this.encoder.getSimState();
    this.encoderSimState.Orientation =
        motorInverted
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    this.systemSim = new LinearSystemSim<>(LinearSystemId.identifyPositionSystem(kV, kA));
  }

  public void updateSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    this.motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    this.encoderSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    this.systemSim.setInput(this.motorSimState.getMotorVoltage());

    // update the models
    this.systemSim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismRadians = this.systemSim.getOutput(0);
    double mechanismRotations = mechanismRadians / (2 * Math.PI);
    double motorRotations = mechanismRotations * this.gearRatio;
    double mechanismRPS = (mechanismRotations - this.prevPosition) / Constants.LOOP_PERIOD_SECS;
    this.prevPosition = mechanismRotations;
    double motorRPS = mechanismRPS * this.gearRatio;
    this.motorSimState.setRawRotorPosition(motorRotations);
    this.motorSimState.setRotorVelocity(motorRPS);
    this.encoderSimState.setRawPosition(mechanismRotations);
    this.encoderSimState.setVelocity(mechanismRPS);
  }
}
