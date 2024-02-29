// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class WheelRadiusCharacterization extends Command {
  private static final TunableNumber characterizationSpeed =
      new TunableNumber("WheelRadiusCharacterization/SpeedRadsPerSec", 0.1);
  private static final double DRIVE_RADIUS =
      Math.hypot(
          RobotConfig.getInstance().getTrackwidth() / 2.0,
          RobotConfig.getInstance().getWheelbase() / 2.0);
  private static final DoubleSupplier gyroYawRadsSupplier =
      () -> RobotOdometry.getInstance().getEstimatedPosition().getRotation().getRadians();

  private final Drivetrain drivetrain;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private double lastGyroYawRads = 0.0;
  private double accumulatedGyroYawRads = 0.0;

  private double[] startWheelPositions;

  private double currentEffectiveWheelRadius = 0.0;

  public WheelRadiusCharacterization(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Reset
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    accumulatedGyroYawRads = 0.0;

    startWheelPositions = drivetrain.getWheelRadiusCharacterizationPosition();

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    // Run drive at velocity
    drivetrain.runWheelRadiusCharacterization(omegaLimiter.calculate(characterizationSpeed.get()));

    // Get yaw and wheel positions
    accumulatedGyroYawRads +=
        MathUtil.angleModulus(gyroYawRadsSupplier.getAsDouble() - lastGyroYawRads);
    lastGyroYawRads = gyroYawRadsSupplier.getAsDouble();
    double averageWheelPosition = 0.0;
    double[] wheelPositions = drivetrain.getWheelRadiusCharacterizationPosition();
    for (int i = 0; i < 4; i++) {
      averageWheelPosition += Math.abs(wheelPositions[i] - startWheelPositions[i]);
    }
    averageWheelPosition /= 4.0;

    currentEffectiveWheelRadius = (accumulatedGyroYawRads * DRIVE_RADIUS) / averageWheelPosition;
    Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/AccumulatedGyroYawRads", accumulatedGyroYawRads);
    Logger.recordOutput(
        "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
        Units.metersToInches(currentEffectiveWheelRadius));
  }

  @Override
  public void end(boolean interrupted) {
    if (accumulatedGyroYawRads <= Math.PI * 2.0) {
      System.out.println("Not enough data for characterization");
    } else {
      System.out.println(
          "Effective Wheel Radius: "
              + Units.metersToInches(currentEffectiveWheelRadius)
              + " inches");
    }
  }
}
