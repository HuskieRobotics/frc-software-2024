package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Field2d;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final TunableNumber topWheelVelocity = new TunableNumber("Shooter/Top Wheel Velocity", 0);
  private final TunableNumber bottomWheelVelocity =
      new TunableNumber("Shooter/Bottom Wheel Velocity", 0);
  private final TunableNumber angle = new TunableNumber("Shooter/Angle", 0);
  private final TunableNumber dunkerWheelVelocity =
      new TunableNumber("Shooter/Dunker Wheel Velocity", 0);

  private ShooterIO io;
  private Alliance alliance = Alliance.Red;
  private InterpolatingDoubleTreeMap angleTreeMap;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) { // TODO: Add intake reference
    this.io = io;
    this.angleTreeMap = new InterpolatingDoubleTreeMap();
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(shooterInputs);
    Logger.processInputs(SUBSYSTEM_NAME, shooterInputs);
    if (TESTING) {
      io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
      io.setShooterWheelTopVelocity(topWheelVelocity.get());
      io.setAngle(angle.get());
    } else {
      this.runAngleStateMachine();
    }
  }

  public void updateAlliance(Alliance newAlliance) {
    this.alliance = newAlliance;
  }

  private void runAngleStateMachine() {
    double test =
        Field2d.getInstance()
            .getAllianceSpeakerCenter()
            .minus(RobotOdometry.getInstance().getEstimatedPosition())
            .getTranslation()
            .getNorm();
    if (RobotOdometry.getInstance().getEstimatedPosition().getX() >= Units.inchesToMeters(421.02)
        && this.alliance == Alliance.Blue) {
      io.setAngle(angleTreeMap.get(test));
    } else if (RobotOdometry.getInstance().getEstimatedPosition().getX()
            <= Units.inchesToMeters(230.2)
        && this.alliance == Alliance.Red) {
      io.setAngle(angleTreeMap.get(RobotOdometry.getInstance().getEstimatedPosition().getX()));
    } else {
      io.setAngle(0);
    }

    this.goToConstantVelocity();
  }

  public void shoot(double topWheelVelocityRPS, double bottomWheelVelocityRPS) {
    // needs to utilize kicker
    io.setShooterWheelTopVelocity(topWheelVelocityRPS);
    io.setShooterWheelBottomVelocity(bottomWheelVelocityRPS);
  }

  public void setAngle(double angle) {
    io.setAngle(angle);
  }

  public void goToConstantVelocity() {
    io.setShooterWheelTopVelocity(topWheelVelocity.get());
    io.setShooterWheelBottomVelocity(bottomWheelVelocity.get());
  }
}
