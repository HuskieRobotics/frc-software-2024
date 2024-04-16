// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

// originally from https://github.com/Mechanical-Advantage/RobotCode2023

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

/**
 * This command, when executed, instructs the drivetrain subsystem to drive to the specified pose in
 * a straight line. The execute method invokes the drivetrain subsystem's drive method. For
 * following a predetermined path, refer to the FollowPath Command class. For generating a path on
 * the fly and following that path, refer to the MoveToPose Command class.
 *
 * <p>Requires: the Drivetrain subsystem
 *
 * <p>Finished When: the robot is at the specified pose (within the specified tolerances)
 *
 * <p>At End: stops the drivetrain
 */
public class DriveToAmp extends DriveToPose {

  /**
   * Constructs a new DriveToPose command that drives the robot in a straight line to the specified
   * pose. A pose supplier is specified instead of a pose since the target pose may not be known
   * when this command is created.
   *
   * @param drivetrain the drivetrain subsystem required by this command
   * @param poseSupplier a supplier that returns the pose to drive to
   */
  public DriveToAmp(Drivetrain drivetrain, Supplier<Pose2d> poseSupplier, Intake intake) {
    super(drivetrain, poseSupplier);
    addRequirements(intake);
  }
}
