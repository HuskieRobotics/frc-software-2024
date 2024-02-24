package frc.lib.team3061.util;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmSystemSim {

  private TalonFX motor;
  private CANcoder encoder;
  private TalonFXSimState motorSimState;
  private CANcoderSimState encoderSimState;
  private SingleJointedArmSim systemSim;
  private double gearRatio;
  private double prevPosition = 0.0;

  public ArmSystemSim(
      TalonFX motor,
      CANcoder encoder,
      boolean motorInverted,
      double gearRatio,
      double length,
      double mass,
      double minAngle,
      double maxAngle,
      double startingAngle) {
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

    this.systemSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                DCMotor.getFalcon500Foc(1),
                SingleJointedArmSim.estimateMOI(length, mass),
                startingAngle),
            DCMotor.getFalcon500Foc(1),
            gearRatio,
            length,
            minAngle,
            maxAngle,
            true,
            startingAngle);
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
    double mechanismRadians = this.systemSim.getAngleRads();
    double mechanismRotations = mechanismRadians / (2 * Math.PI);
    double motorRotations = mechanismRotations * this.gearRatio;
    double mechanismRadiansPerSec = this.systemSim.getVelocityRadPerSec();
    double mechanismRPS = mechanismRadiansPerSec / (2 * Math.PI);
    double motorRPS = mechanismRPS * this.gearRatio;
    this.motorSimState.setRawRotorPosition(motorRotations);
    this.motorSimState.setRotorVelocity(motorRPS);
    this.encoderSimState.setRawPosition(mechanismRotations);
    this.encoderSimState.setVelocity(mechanismRPS);
  }
}
