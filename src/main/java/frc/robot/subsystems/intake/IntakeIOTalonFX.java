package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX rightRollerMotor;
  private TalonFX leftRollerMotor;
  private TalonFX drumMotor;
  private DigitalInput rightRollerIRSensor;
  private DigitalInput leftRollerIRSensor;
  private DigitalInput drumIRSensor;

  public IntakeIOTalonFX() {
    rightRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_RIGHT_ROLLER_IR_SENSOR_ID);
    leftRollerIRSensor = new DigitalInput(IntakeConstants.INTAKE_LEFT_ROLLER_IR_SENSOR_ID);
    drumIRSensor = new DigitalInput(IntakeConstants.INTAKE_DRUM_IR_SENSOR_ID);

    configureIntakeMotors();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.isRightRollerIRBlocked = rightRollerIRSensor.get();
    inputs.isLeftRollerIRBlocked = leftRollerIRSensor.get();
    inputs.isDrumIRBlocked = drumIRSensor.get();

    inputs.rightRollerAppliedPercentage = rightRollerMotor.getDutyCycle().getValueAsDouble();
    inputs.leftRollerAppliedPercentage = leftRollerMotor.getDutyCycle().getValueAsDouble();
    inputs.drumAppliedPercentage = drumMotor.getDutyCycle().getValueAsDouble();

    inputs.rightRollerStatorCurrentAmps =
        new double[] {rightRollerMotor.getStatorCurrent().getValueAsDouble()};
    inputs.leftRollerStatorCurrentAmps =
        new double[] {leftRollerMotor.getStatorCurrent().getValueAsDouble()};
    inputs.drumStatorCurrentAmps = new double[] {drumMotor.getStatorCurrent().getValueAsDouble()};
  }

  @Override
  public void setRightRollerPower(double percentage) {
    rightRollerMotor.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setLeftRollerPower(double percentage) {
    leftRollerMotor.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setDrumPower(double percentage) {
    drumMotor.setControl(new DutyCycleOut(percentage));
  }

  @Override
  public void setRightRollerCurrent(double current) {
    rightRollerMotor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void setLeftRollerCurrent(double current) {
    leftRollerMotor.setControl(new TorqueCurrentFOC(current));
  }

  @Override
  public void setDrumCurrent(double current) {
    drumMotor.setControl(new TorqueCurrentFOC(current));
  }

  private void configureIntakeMotors() {
    // might need to invert motors based on cad
    rightRollerMotor = new TalonFX(IntakeConstants.INTAKE_RIGHT_ROLLER_MOTOR_ID);
    leftRollerMotor = new TalonFX(IntakeConstants.INTAKE_LEFT_ROLLER_MOTOR_ID);
    drumMotor = new TalonFX(IntakeConstants.INTAKE_DRUM_MOTOR_ID);
  }
}
