package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.lib.team3061.RobotConfig;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rightRollerMotor;
  private TalonFX leftRollerMotor;
  private TalonFX drumMotor;
  private final DigitalInput rightRollerIRSensor;
  private final DigitalInput leftRollerIRSensor;
  private final DigitalInput drumIRSensor;

  private double rightRollerRequestedVelocity;
  private double leftRollerRequestedVelocity;
  private double drumRequestedVelocity;

  private VelocityTorqueCurrentFOC rightRollerVelocityRequest;
  private VelocityTorqueCurrentFOC leftRollerVelocityRequest;
  private VelocityTorqueCurrentFOC drumVelocityRequest;

  private StatusSignal<Double> rightRollerVelocityStatusSignal;
  private StatusSignal<Double> leftRollerVelocityStatusSignal;
  private StatusSignal<Double> drumVelocityStatusSignal;
  // status signals for stator current
  private StatusSignal<Double> rightRollerStatorCurrentStatusSignal;
  private StatusSignal<Double> leftRollerStatorCurrentStatusSignal;
  private StatusSignal<Double> drumStatorCurrentStatusSignal;

  private StatusSignal<Double> rightRollerSupplyCurrentStatusSignal;
  private StatusSignal<Double> leftRollerSupplyCurrentStatusSignal;
  private StatusSignal<Double> drumSupplyCurrentStatusSignal;

  // simulation related
  private TalonFXSimState rightRollerMotorSimState;
  private TalonFXSimState leftRollerMotorSimState;
  private TalonFXSimState drumMotorSimState;
  private LinearSystemSim<N1, N1, N1> rightRollerSim;
  private LinearSystemSim<N1, N1, N1> leftRollerSim;
  private LinearSystemSim<N1, N1, N1> drumSim;
  private VelocityVoltage rightRollerVelocitySimRequest;
  private VelocityVoltage leftRollerVelocitySimRequest;
  private VelocityVoltage drumVelocitySimRequest;

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

  public IntakeIOTalonFX() {
    rightRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_RIGHT_ROLLER_IR_SENSOR_ID);
    leftRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_LEFT_ROLLER_IR_SENSOR_ID);
    drumIRSensor = new DigitalInput(IntakeConstants.INTAKE_DRUM_IR_SENSOR_ID);

    // torque current FOC control mode is not support in simulation yet
    rightRollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    leftRollerVelocityRequest = new VelocityTorqueCurrentFOC(0);
    drumVelocityRequest = new VelocityTorqueCurrentFOC(0);

    rightRollerVelocityStatusSignal = rightRollerMotor.getRotorVelocity();
    leftRollerVelocityStatusSignal = leftRollerMotor.getRotorVelocity();
    drumVelocityStatusSignal = drumMotor.getRotorVelocity();

    rightRollerStatorCurrentStatusSignal = rightRollerMotor.getStatorCurrent();
    leftRollerStatorCurrentStatusSignal = leftRollerMotor.getStatorCurrent();
    drumStatorCurrentStatusSignal = drumMotor.getStatorCurrent();

    rightRollerSupplyCurrentStatusSignal = rightRollerMotor.getSupplyCurrent();
    leftRollerSupplyCurrentStatusSignal = leftRollerMotor.getSupplyCurrent();
    drumSupplyCurrentStatusSignal = drumMotor.getSupplyCurrent();

    configureIntakeRollerMotors(
        IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_ID, IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_ID);
    configureIntakeDrumMotor(IntakeConstants.INTAKE_DRUM_MOTOR_ID);

    configSim();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // FIXME: specify gear ratios
    updateSim(this.rightRollerMotorSimState, this.rightRollerSim, 1.0);
    updateSim(this.leftRollerMotorSimState, this.leftRollerSim, 1.0);
    updateSim(this.drumMotorSimState, this.drumSim, 1.0);

    BaseStatusSignal.refreshAll(
        rightRollerVelocityStatusSignal,
        leftRollerVelocityStatusSignal,
        drumVelocityStatusSignal,
        rightRollerStatorCurrentStatusSignal,
        leftRollerStatorCurrentStatusSignal,
        drumStatorCurrentStatusSignal,
        rightRollerSupplyCurrentStatusSignal,
        leftRollerSupplyCurrentStatusSignal,
        drumSupplyCurrentStatusSignal);

    inputs.isRightRollerIRBlocked = rightRollerIRSensor.get();
    inputs.isLeftRollerIRBlocked = leftRollerIRSensor.get();
    inputs.isDrumIRBlocked = drumIRSensor.get();

    inputs.rightRollerStatorCurrentAmps = rightRollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.leftRollerStatorCurrentAmps = leftRollerStatorCurrentStatusSignal.getValueAsDouble();
    inputs.drumStatorCurrentAmps = drumStatorCurrentStatusSignal.getValueAsDouble();

    inputs.rightRollerSupplyCurrentAmps = rightRollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.leftRollerSupplyCurrentAmps = leftRollerSupplyCurrentStatusSignal.getValueAsDouble();
    inputs.drumSupplyCurrentAmps = drumSupplyCurrentStatusSignal.getValueAsDouble();

    inputs.rightRollerVelocityRotationsPerSecond =
        rightRollerVelocityStatusSignal.getValueAsDouble();
    inputs.leftRollerVelocityRotationsPerSecond = leftRollerVelocityStatusSignal.getValueAsDouble();
    inputs.drumVelocityRotationsPerSecond = drumVelocityStatusSignal.getValueAsDouble();

    inputs.rightRollerReferenceVelocityRPS = rightRollerRequestedVelocity;
    inputs.leftRollerReferenceVelocityRPS = leftRollerRequestedVelocity;
    inputs.drumReferenceVelocityRPS = drumRequestedVelocity;
  }

  @Override
  public void setRightRollerVelocity(double rps) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      rightRollerMotor.setControl(rightRollerVelocityRequest.withVelocity(rps));
    } else {
      rightRollerMotor.setControl(rightRollerVelocitySimRequest.withVelocity(rps));
    }
  }

  @Override
  public void setLeftRollerVelocity(double rps) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      leftRollerMotor.setControl(leftRollerVelocityRequest.withVelocity(rps));
    } else {
      leftRollerMotor.setControl(leftRollerVelocitySimRequest.withVelocity(rps));
    }
  }

  @Override
  public void setDrumVelocity(double rps) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      drumMotor.setControl(drumVelocityRequest.withVelocity(rps));
    } else {
      drumMotor.setControl(drumVelocitySimRequest.withVelocity(rps));
    }
  }

  private void configureIntakeRollerMotors(int right_id, int left_id) {
    rightRollerMotor = new TalonFX(right_id, RobotConfig.getInstance().getCANBusName());
    leftRollerMotor = new TalonFX(left_id, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration rollerConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs rollerCurrentLimits = new CurrentLimitsConfigs();

    rollerCurrentLimits.SupplyCurrentLimit = IntakeConstants.ROLLERS_CONTINUOUS_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyCurrentThreshold = IntakeConstants.ROLLERS_PEAK_CURRENT_LIMIT;
    rollerCurrentLimits.SupplyTimeThreshold = IntakeConstants.ROLLERS_PEAK_CURRENT_DURATION;
    rollerCurrentLimits.SupplyCurrentLimitEnable = true;

    rollerConfig.CurrentLimits = rollerCurrentLimits;

    rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // FIXME: need different values for simulation; hard code?
    rollerConfig.Slot0.kP = rollerMotorsKP.get();
    rollerConfig.Slot0.kI = rollerMotorsKI.get();
    rollerConfig.Slot0.kD = rollerMotorsKD.get();
    rollerConfig.Slot0.kS = rollerMotorsKS.get();

    rollerConfig.MotorOutput.Inverted =
        IntakeConstants.ROLLERS_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    rightRollerMotor.getConfigurator().apply(rollerConfig);
    leftRollerMotor.getConfigurator().apply(rollerConfig);
  }

  private void configureIntakeDrumMotor(int drum_id) {
    drumMotor = new TalonFX(drum_id, RobotConfig.getInstance().getCANBusName());

    TalonFXConfiguration drumConfig = new TalonFXConfiguration();
    CurrentLimitsConfigs drumCurrentLimits = new CurrentLimitsConfigs();

    drumCurrentLimits.SupplyCurrentLimit = IntakeConstants.DRUM_CONTINUOUS_CURRENT_LIMIT;
    drumCurrentLimits.SupplyCurrentThreshold = IntakeConstants.DRUM_PEAK_CURRENT_LIMIT;
    drumCurrentLimits.SupplyTimeThreshold = IntakeConstants.DRUM_PEAK_CURRENT_DURATION;
    drumCurrentLimits.SupplyCurrentLimitEnable = true;

    drumConfig.CurrentLimits = drumCurrentLimits;

    drumConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // FIXME: need different values for simulation; hard code?
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

  private void configSim() {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    this.rightRollerMotorSimState = this.rightRollerMotor.getSimState();
    this.rightRollerMotorSimState.Orientation =
        IntakeConstants.ROLLERS_MOTOR_INVERTED
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    this.leftRollerMotorSimState = this.leftRollerMotor.getSimState();
    this.leftRollerMotorSimState.Orientation =
        IntakeConstants.ROLLERS_MOTOR_INVERTED
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;
    this.drumMotorSimState = this.drumMotor.getSimState();
    this.drumMotorSimState.Orientation =
        IntakeConstants.DRUM_MOTOR_INVERTED
            ? ChassisReference.Clockwise_Positive
            : ChassisReference.CounterClockwise_Positive;

    // FIXME: characterize the system to obtain the kV and kA values (use recalc)
    this.rightRollerSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(2.0, 0.2));
    this.leftRollerSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(2.0, 0.2));
    this.drumSim = new LinearSystemSim<>(LinearSystemId.identifyVelocitySystem(2.0, 0.2));

    this.rightRollerVelocitySimRequest = new VelocityVoltage(0);
    this.leftRollerVelocitySimRequest = new VelocityVoltage(0);
    this.drumVelocitySimRequest = new VelocityVoltage(0);
  }

  private void updateSim(
      TalonFXSimState simState, LinearSystemSim<N1, N1, N1> sim, double gearRatio) {
    if (Constants.getMode() != Constants.Mode.SIM) {
      return;
    }

    // update the sim states supply voltage based on the simulated battery
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    // update the input voltages of the models based on the outputs of the simulated TalonFXs
    sim.setInput(simState.getMotorVoltage());

    // update the models
    sim.update(Constants.LOOP_PERIOD_SECS);

    // update the simulated TalonFX based on the model outputs
    double mechanismRadiansPerSec = sim.getOutput(0);
    double motorRPS = mechanismRadiansPerSec * gearRatio / (2 * Math.PI);
    double motorRotations = motorRPS * Constants.LOOP_PERIOD_SECS;
    simState.addRotorPosition(motorRotations);
    simState.setRotorVelocity(motorRPS);
  }
}
