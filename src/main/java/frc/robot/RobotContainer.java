// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.Drivetrain;
import frc.lib.team3061.drivetrain.DrivetrainIO;
import frc.lib.team3061.drivetrain.DrivetrainIOCTRE;
import frc.lib.team3061.drivetrain.DrivetrainIOGeneric;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIO;
import frc.lib.team3061.drivetrain.swerve.SwerveModuleIOTalonFXPhoenix6;
import frc.lib.team3061.gyro.GyroIOPigeon2Phoenix6;
import frc.lib.team3061.leds.LEDs;
import frc.lib.team3061.leds.LEDs.ShooterLEDState;
import frc.lib.team3061.vision.Vision;
import frc.lib.team3061.vision.VisionConstants;
import frc.lib.team3061.vision.VisionIO;
import frc.lib.team3061.vision.VisionIOPhotonVision;
import frc.lib.team3061.vision.VisionIOSim;
import frc.lib.team6328.util.NoteVisualizer;
import frc.robot.Constants.Mode;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopSwerveAimAtSpeaker;
import frc.robot.commands.TeleopSwerveAimToPass;
import frc.robot.configs.ArtemisRobotConfig;
import frc.robot.configs.GenericDrivetrainRobotConfig;
import frc.robot.configs.PracticeBoardConfig;
import frc.robot.configs.PracticeRobotConfig;
import frc.robot.operator_interface.OISelector;
import frc.robot.operator_interface.OperatorInterface;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.ShootingPosition;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private OperatorInterface oi = new OperatorInterface() {};
  private RobotConfig config;
  private Drivetrain drivetrain;
  private Alliance lastAlliance = DriverStation.Alliance.Red;
  private Vision vision;
  private Shooter shooter;
  private Intake intake;

  // use AdvantageKit's LoggedDashboardChooser instead of SendableChooser to ensure accurate logging
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Routine");

  private final LoggedDashboardNumber endgameAlert1 =
      new LoggedDashboardNumber("Endgame Alert #1", 20.0);
  private final LoggedDashboardNumber endgameAlert2 =
      new LoggedDashboardNumber("Endgame Alert #2", 10.0);

  // RobotContainer singleton
  private static RobotContainer robotContainer = new RobotContainer();

  /**
   * Create the container for the robot. Contains subsystems, operator interface (OI) devices, and
   * commands.
   */
  public RobotContainer() {
    /*
     * IMPORTANT: The RobotConfig subclass object *must* be created before any other objects
     * that use it directly or indirectly. If this isn't done, a null pointer exception will result.
     */
    createRobotConfig();

    LEDs.getInstance();

    // create real, simulated, or replay subsystems based on the mode and robot specified
    if (Constants.getMode() != Mode.REPLAY) {

      switch (Constants.getRobot()) {
        case ROBOT_PRACTICE_BOARD:
          {
            createPracticeBoardSubsystem();
            break;
          }
        case ROBOT_PRACTICE:
        case ROBOT_COMPETITION:
          {
            createCTRESubsystems();
            break;
          }
        case ROBOT_SIMBOT:
          {
            createSubsystems();
            break;
          }
        case ROBOT_SIMBOT_CTRE:
          {
            createCTRESimSubsystems();

            break;
          }
        default:
          break;
      }

    } else {
      drivetrain = new Drivetrain(new DrivetrainIO() {});

      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {}, intake, drivetrain);
      intake.setShooterAngleReady(shooter.getShooterAngleReadySupplier());

      String[] cameraNames = config.getCameraNames();
      VisionIO[] visionIOs = new VisionIO[cameraNames.length];
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIO() {};
      }
      vision = new Vision(visionIOs);
    }

    // disable all telemetry in the LiveWindow to reduce the processing during each iteration
    LiveWindow.disableAllTelemetry();

    constructField();

    updateOI();

    configureAutoCommands();
  }

  /**
   * The RobotConfig subclass object *must* be created before any other objects that use it directly
   * or indirectly. If this isn't done, a null pointer exception will result.
   */
  private void createRobotConfig() {
    switch (Constants.getRobot()) {
      case ROBOT_SIMBOT:
        config = new GenericDrivetrainRobotConfig();
        break;
      case ROBOT_PRACTICE:
        config = new PracticeRobotConfig();
        break;
      case ROBOT_COMPETITION:
      case ROBOT_SIMBOT_CTRE:
        config = new ArtemisRobotConfig();
        break;
      case ROBOT_PRACTICE_BOARD:
        config = new PracticeBoardConfig();
    }
  }

  private void createCTRESubsystems() {
    drivetrain = new Drivetrain(new DrivetrainIOCTRE());

    intake = new Intake(new IntakeIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX(), intake, drivetrain);
    intake.setShooterAngleReady(shooter.getShooterAngleReadySupplier());

    String[] cameraNames = config.getCameraNames();
    VisionIO[] visionIOs = new VisionIO[cameraNames.length];
    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }
    for (int i = 0; i < visionIOs.length; i++) {
      visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout);
    }
    vision = new Vision(visionIOs);
  }

  private void createSubsystems() {
    int[] driveMotorCANIDs = config.getSwerveDriveMotorCANIDs();
    int[] steerMotorCANDIDs = config.getSwerveSteerMotorCANIDs();
    int[] steerEncoderCANDIDs = config.getSwerveSteerEncoderCANIDs();
    double[] steerOffsets = config.getSwerveSteerOffsets();
    SwerveModuleIO flModule =
        new SwerveModuleIOTalonFXPhoenix6(
            0, driveMotorCANIDs[0], steerMotorCANDIDs[0], steerEncoderCANDIDs[0], steerOffsets[0]);

    SwerveModuleIO frModule =
        new SwerveModuleIOTalonFXPhoenix6(
            1, driveMotorCANIDs[1], steerMotorCANDIDs[1], steerEncoderCANDIDs[1], steerOffsets[1]);

    SwerveModuleIO blModule =
        new SwerveModuleIOTalonFXPhoenix6(
            2, driveMotorCANIDs[2], steerMotorCANDIDs[2], steerEncoderCANDIDs[2], steerOffsets[2]);

    SwerveModuleIO brModule =
        new SwerveModuleIOTalonFXPhoenix6(
            3, driveMotorCANIDs[3], steerMotorCANDIDs[3], steerEncoderCANDIDs[3], steerOffsets[3]);

    drivetrain =
        new Drivetrain(
            new DrivetrainIOGeneric(
                new GyroIOPigeon2Phoenix6(config.getGyroCANID()),
                flModule,
                frModule,
                blModule,
                brModule));

    intake = new Intake(new IntakeIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX(), intake, drivetrain);
    intake.setShooterAngleReady(shooter.getShooterAngleReadySupplier());

    if (Constants.getRobot() == Constants.RobotType.ROBOT_SIMBOT) {
      vision = new Vision(new VisionIO[] {new VisionIO() {}});
    } else {
      String[] cameraNames = config.getCameraNames();
      VisionIO[] visionIOs = new VisionIO[cameraNames.length];
      AprilTagFieldLayout layout;
      try {
        layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
      } catch (IOException e) {
        layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
      }
      for (int i = 0; i < visionIOs.length; i++) {
        visionIOs[i] = new VisionIOPhotonVision(cameraNames[i], layout);
      }
      vision = new Vision(visionIOs);
    }
  }

  private void createCTRESimSubsystems() {
    DrivetrainIO drivetrainIO = new DrivetrainIOCTRE();
    drivetrain = new Drivetrain(drivetrainIO);

    intake = new Intake(new IntakeIOTalonFX());
    shooter = new Shooter(new ShooterIOTalonFX(), intake, drivetrain);
    intake.setShooterAngleReady(shooter.getShooterAngleReadySupplier());

    // vision = new Vision(new VisionIO[] {new VisionIO() {}});

    AprilTagFieldLayout layout;
    try {
      layout = new AprilTagFieldLayout(VisionConstants.APRILTAG_FIELD_LAYOUT_PATH);
    } catch (IOException e) {
      layout = new AprilTagFieldLayout(new ArrayList<>(), 16.4592, 8.2296);
    }
    vision =
        new Vision(
            new VisionIO[] {
              new VisionIOSim(
                  layout,
                  drivetrain::getPose,
                  RobotConfig.getInstance().getRobotToCameraTransforms()[0])
            });
  }

  private void createPracticeBoardSubsystem() {
    // change the following to connect the subsystem being tested to actual hardware
    drivetrain = new Drivetrain(new DrivetrainIO() {});
    intake = new Intake(new IntakeIO() {});
    shooter = new Shooter(new ShooterIO() {}, intake, drivetrain);
    intake.setShooterAngleReady(shooter.getShooterAngleReadySupplier());
    vision = new Vision(new VisionIO[] {new VisionIO() {}});
  }

  /**
   * Creates the field from the defined regions and transition points from one region to its
   * neighbor. The field is used to generate paths.
   */
  private void constructField() {
    Field2d.getInstance().setRegions(new Region2d[] {});
  }

  /**
   * This method scans for any changes to the connected operator interface (e.g., joysticks). If
   * anything changed, it creates a new OI object and binds all of the buttons to commands.
   */
  public void updateOI() {
    OperatorInterface prevOI = oi;
    oi = OISelector.getOperatorInterface();
    if (oi == prevOI) {
      return;
    }

    configureButtonBindings();
  }

  /**
   * Factory method to create the singleton robot container object.
   *
   * @return the singleton robot container object
   */
  public static RobotContainer getInstance() {
    return robotContainer;
  }

  /** Use this method to define your button->command mappings. */
  private void configureButtonBindings() {

    configureDrivetrainCommands();

    configureIntakeCommands();

    configureVisionCommands();

    configureShooterCommands();

    // Endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(
            Commands.run(() -> LEDs.getInstance().setEndgameAlert(true))
                .withTimeout(1)
                .andThen(
                    Commands.run(() -> LEDs.getInstance().setEndgameAlert(false))
                        .withTimeout(1.0)));
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0.0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(false)).withTimeout(0.25),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(true)).withTimeout(0.5),
                Commands.run(() -> LEDs.getInstance().setEndgameAlert(false)).withTimeout(0.25)));

    // interrupt all commands by running a command that requires every subsystem. This is used to
    // recover to a known state if the robot becomes "stuck" in a command.
    oi.getInterruptAll()
        .onTrue(
            Commands.parallel(
                Commands.runOnce(drivetrain::disableXstance),
                new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate)));
  }

  /** Use this method to define your commands for autonomous mode. */
  private void configureAutoCommands() {
    // Waypoints
    NamedCommands.registerCommand("command1", Commands.print("passed marker 1"));
    NamedCommands.registerCommand("command2", Commands.print("passed marker 2"));
    NamedCommands.registerCommand(
        "enableXStance", Commands.runOnce(drivetrain::enableXstance, drivetrain));
    NamedCommands.registerCommand(
        "disableXStance", Commands.runOnce(drivetrain::disableXstance, drivetrain));
    NamedCommands.registerCommand("wait5Seconds", Commands.waitSeconds(5.0));

    NamedCommands.registerCommand("Shoot Now", getAutoShootNowCommand());
    NamedCommands.registerCommand("Stop And Shoot", getAutoStopAndShootCommand());

    // build auto path commands

    // add commands to the auto chooser
    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    /************ 1 Note Source Side ************
     *
     * shoot initial note and leave robot starting zone
     *
     */
    Command oneNoteSourceSide =
        Commands.sequence(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_AUTO)),
            new PathPlannerAuto("1 Note Auto"));
    autoChooser.addOption("1 Note Source Side", oneNoteSourceSide);

    /************ 4 Note Source Side ************
     *
     * 4 notes (initial and first three center notes from source side)
     *
     */

    Command fourNoteSourceSideBlue =
        Commands.sequence(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_AUTO)),
            new PathPlannerAuto("Collect 2nd"),
            Commands.either(
                new PathPlannerAuto("Score 2nd Collect 3rd"),
                new PathPlannerAuto("Missed 2nd Collect 3rd"),
                intake::hasNoteForAuto),
            Commands.runOnce(
                () -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_UNDER_STAGE_AUTO)),
            Commands.either(
                new PathPlannerAuto("Score 3rd Collect 4th"),
                new PathPlannerAuto("Missed 3rd Collect 4th"),
                intake::hasNoteForAuto),
            Commands.runOnce(
                () -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_UNDER_STAGE_AUTO)),
            new PathPlannerAuto("Score 4th Center"));
    autoChooser.addOption("4 Note Source Side Blue", fourNoteSourceSideBlue);

    Command fourNoteSourceSideRed =
        Commands.sequence(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_AUTO)),
            new PathPlannerAuto("Collect 2nd Red"),
            Commands.either(
                new PathPlannerAuto("Score 2nd Collect 3rd Red"),
                new PathPlannerAuto("Missed 2nd Collect 3rd Red"),
                intake::hasNoteForAuto),
            Commands.runOnce(
                () -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_UNDER_STAGE_AUTO)),
            Commands.either(
                new PathPlannerAuto("Score 3rd Collect 4th Red"),
                new PathPlannerAuto("Missed 3rd Collect 4th Red"),
                intake::hasNoteForAuto),
            Commands.runOnce(
                () -> shooter.setShootingPosition(ShootingPosition.SOURCE_SIDE_UNDER_STAGE_AUTO)),
            new PathPlannerAuto("Score 4th Center"));
    autoChooser.addOption("4 Note Source Side Red", fourNoteSourceSideRed);

    /************ 6 Note Amp Side ************
     *
     * 6 notes (initial, first two center notes from amp side, three notes near the speaker)
     *
     */

    Command sixNoteAmpSide =
        Commands.sequence(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.AMP_SIDE_AUTO)),
            new PathPlannerAuto("Amp Collect 2nd"),
            Commands.either(
                new PathPlannerAuto("Amp Score 2nd Collect 3rd"),
                new PathPlannerAuto("Amp Missed 2nd Collect 3rd"),
                intake::hasNoteForAuto),
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SUBWOOFER)),
            new PathPlannerAuto("Amp Score 3rd Collect 4th"),
            new PathPlannerAuto("4 note center"));

    autoChooser.addOption("6 Note Amp Side", sixNoteAmpSide);

    /************ 5 Note Amp Side ************
     *
     * 5 notes (initial, first or second center note from amp side, three notes near the speaker)
     *
     */
    Command fiveNoteAmpSide =
        Commands.sequence(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.AMP_SIDE_AUTO)),
            new PathPlannerAuto("Amp Collect 2nd"),
            Commands.either(
                Commands.sequence(
                    Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SUBWOOFER)),
                    new PathPlannerAuto("Amp Score 2nd Skip 3rd Collect 4th")),
                Commands.sequence(
                    new PathPlannerAuto("Amp Missed 2nd Collect 3rd"),
                    Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SUBWOOFER)),
                    new PathPlannerAuto("Amp Score 3rd Collect 4th")),
                intake::hasNoteForAuto),
            new PathPlannerAuto("4 note center"));

    autoChooser.addOption("5 Note Amp Side", fiveNoteAmpSide);

    /************ 4 Note Near Speaker ************
     *
     * 4 note (initial and three notes near the speaker)
     *
     */
    Command fourNoteCenter =
        Commands.parallel(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SUBWOOFER)),
            new PathPlannerAuto("4 note center slow"));
    autoChooser.addOption("4 Note Center", fourNoteCenter);

    /************ Start Point ************
     *
     * useful for initializing the pose of the robot to a known location
     *
     */

    Command startPoint =
        Commands.runOnce(
            () ->
                drivetrain.resetPose(
                    PathPlannerPath.fromPathFile("Start Point").getPreviewStartingHolonomicPose()),
            drivetrain);
    autoChooser.addOption("Start Point", startPoint);

    /************ Distance Test ************
     *
     * used for empirically determining the wheel diameter
     *
     */
    Command distanceTestPathCommand = new PathPlannerAuto("DistanceTest");
    autoChooser.addOption("Distance Path", distanceTestPathCommand);

    /************ Auto Tuning ************
     *
     * useful for tuning the autonomous PID controllers
     *
     */
    Command tuningCommand = new PathPlannerAuto("Tuning");
    autoChooser.addOption("Auto Tuning", tuningCommand);

    /************ Drive Velocity Tuning ************
     *
     * useful for tuning the drive velocity PID controller
     *
     */
    autoChooser.addOption(
        "Drive Velocity Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
            Commands.repeatingSequence(
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(2.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(-0.5, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(1.0),
                    Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(3.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> drivetrain.drive(1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(() -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-3.0, 0.0, 0.0, false, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(2.0),
                    Commands.run(
                        () -> drivetrain.drive(-1.0, 0.0, 0.0, false, false), drivetrain)))));

    /************ Swerve Rotation Tuning ************
     *
     * useful for tuning the swerve module rotation PID controller
     *
     */
    autoChooser.addOption(
        "Swerve Rotation Tuning",
        Commands.sequence(
            Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
            Commands.repeatingSequence(
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(0.1, 0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-0.1, 0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(() -> drivetrain.drive(-0.1, -0.1, 0.0, true, false), drivetrain)),
                Commands.deadline(
                    Commands.waitSeconds(0.5),
                    Commands.run(
                        () -> drivetrain.drive(0.1, -0.1, 0.0, true, false), drivetrain)))));

    Shuffleboard.getTab("MAIN").add(autoChooser.getSendableChooser());
  }

  private void configureIntakeCommands() {
    oi.getIntakeAutomationSwitch()
        .onTrue(
            Commands.parallel(
                    Commands.runOnce(shooter::intakeEnabled),
                    Commands.runOnce(intake::enableAutomation, intake))
                .ignoringDisable(true)
                .withName("enable intake automation"));

    oi.getIntakeAutomationSwitch()
        .onFalse(
            Commands.parallel(
                    Commands.runOnce(intake::disableAutomation, intake),
                    Commands.runOnce(intake::turnIntakeOff),
                    Commands.runOnce(intake::turnKickerOff),
                    Commands.runOnce(shooter::intakeDisabled))
                .ignoringDisable(true)
                .withName("disable intake automation"));

    Trigger coastModeButton = new Trigger(shooter::getCoastEnableOverride);
    coastModeButton.onTrue(
        Commands.runOnce(() -> shooter.setCoastModeOverride(true)).ignoringDisable(true));
    coastModeButton.onFalse(
        Commands.runOnce(() -> shooter.setCoastModeOverride(false)).ignoringDisable(true));

    oi.getRunIntakeButton()
        .and(() -> !intake.automationEnabled())
        .whileTrue(
            Commands.parallel(
                    Commands.run(intake::intakeGamePiece),
                    Commands.run(intake::transitionGamePiece))
                .withName("ManualIntakeOn"));

    oi.getRunIntakeButton()
        .and(() -> !intake.automationEnabled())
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(intake::turnIntakeOff),
                    Commands.runOnce(intake::turnKickerOff))
                .withName("ManualIntakeOff"));

    oi.getOuttakeAllButton()
        .and(() -> !intake.automationEnabled())
        .whileTrue(Commands.run(intake::outtakeAll).withName("ManualOuttakeOn"));

    oi.getOuttakeAllButton()
        .and(() -> !intake.automationEnabled())
        .onFalse(
            Commands.sequence(
                    Commands.runOnce(intake::turnIntakeOff),
                    Commands.runOnce(intake::turnKickerOff))
                .withName("ManualOuttakeOff"));
  }

  private void configureDrivetrainCommands() {
    /*
     * Set up the default command for the drivetrain. The joysticks' values map to percentage of the
     * maximum velocities. The velocities may be specified from either the robot's frame of
     * reference or the field's frame of reference. In the robot's frame of reference, the positive
     * x direction is forward; the positive y direction, left; position rotation, CCW. In the field
     * frame of reference, the origin of the field to the lower left corner (i.e., the corner of the
     * field to the driver's right). Zero degrees is away from the driver and increases in the CCW
     * direction. This is why the left joystick's y axis specifies the velocity in the x direction
     * and the left joystick's x axis specifies the velocity in the y direction.
     */
    drivetrain.setDefaultCommand(
        new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, oi::getRotate));

    // lock rotation to the nearest 180° while driving
    oi.getLock180Button()
        .whileTrue(
            new TeleopSwerve(
                    drivetrain,
                    oi::getTranslateX,
                    oi::getTranslateY,
                    () ->
                        (drivetrain.getPose().getRotation().getDegrees() > -90
                                && drivetrain.getPose().getRotation().getDegrees() < 90)
                            ? Rotation2d.fromDegrees(0.0)
                            : Rotation2d.fromDegrees(180.0))
                .withName("lock 180"));

    oi.getAimSpeakerButton()
        .toggleOnTrue(
            Commands.parallel(
                new TeleopSwerveAimAtSpeaker(
                    drivetrain, shooter, intake, oi::getTranslateX, oi::getTranslateY),
                Commands.runOnce(
                    () -> shooter.setShootingPosition(ShootingPosition.FIELD), shooter)));

    oi.getAimAndShootSpeakerButton()
        .toggleOnTrue(
            Commands.parallel(
                new TeleopSwerveAimAtSpeaker(
                    drivetrain, shooter, intake, oi::getTranslateX, oi::getTranslateY),
                Commands.sequence(
                    Commands.runOnce(
                        () -> shooter.setShootingPosition(ShootingPosition.AUTO_SHOT), shooter),
                    getShootCommand())));

    // field-relative toggle
    oi.getFieldRelativeButton()
        .toggleOnTrue(
            Commands.either(
                    Commands.runOnce(drivetrain::disableFieldRelative, drivetrain),
                    Commands.runOnce(drivetrain::enableFieldRelative, drivetrain),
                    drivetrain::getFieldRelative)
                .withName("toggle field relative"));

    // slow-mode toggle
    oi.getTranslationSlowModeButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableTranslationSlowMode, drivetrain)
                .withName("enable translation slow mode"));
    oi.getTranslationSlowModeButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableTranslationSlowMode, drivetrain)
                .withName("disable translation slow mode"));
    oi.getRotationSlowModeButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableRotationSlowMode, drivetrain)
                .withName("enable rotation slow mode"));
    oi.getRotationSlowModeButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableRotationSlowMode, drivetrain)
                .withName("disable rotation slow mode"));

    // reset gyro to 0 degrees
    oi.getResetGyroButton()
        .onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain).withName("zero gyro"));

    // reset pose based on vision
    oi.getResetPoseToVisionButton()
        .onTrue(
            Commands.repeatingSequence(Commands.none())
                .until(() -> vision.getBestRobotPose() != null)
                .andThen(
                    Commands.runOnce(
                        () -> drivetrain.resetPoseToVision(() -> vision.getBestRobotPose())))
                .ignoringDisable(true)
                .withName("reset pose to vision"));

    // x-stance
    oi.getXStanceButton()
        .onTrue(
            Commands.runOnce(drivetrain::enableXstance, drivetrain).withName("enable x-stance"));
    oi.getXStanceButton()
        .onFalse(
            Commands.runOnce(drivetrain::disableXstance, drivetrain).withName("disable x-stance"));

    // turbo
    oi.getTurboButton()
        .onTrue(Commands.runOnce(drivetrain::enableTurbo, drivetrain).withName("enable turbo"));
    oi.getTurboButton()
        .onFalse(Commands.runOnce(drivetrain::disableTurbo, drivetrain).withName("disable turbo"));
  }

  private void configureVisionCommands() {
    // enable/disable vision
    oi.getVisionIsEnabledSwitch()
        .onTrue(
            Commands.runOnce(() -> vision.enable(true))
                .ignoringDisable(true)
                .withName("enable vision"));
    oi.getVisionIsEnabledSwitch()
        .onFalse(
            Commands.parallel(
                    Commands.runOnce(() -> vision.enable(false), vision),
                    Commands.runOnce(drivetrain::resetPoseRotationToGyro))
                .ignoringDisable(true)
                .withName("disable vision"));
  }

  private void configureShooterCommands() {
    NoteVisualizer.setRobotPoseSupplier(this.drivetrain::getPose);

    oi.getAimAutomationSwitch()
        .onTrue(
            Commands.runOnce(shooter::enableAutomatedShooter, shooter)
                .ignoringDisable(true)
                .withName("enable shooter automation"));
    oi.getAimAutomationSwitch()
        .onFalse(
            Commands.runOnce(shooter::disableAutomatedShooter, shooter)
                .ignoringDisable(true)
                .withName("disable shooter automation"));

    oi.getPrepareToScoreAmpButton()
        .onTrue(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.AMP), shooter)
                .withName("prepare to score amp")
                .until(
                    () -> {
                      return !intake.hasNote();
                    }));

    // oi.getPrepareToScoreAmpButton()
    //     .onTrue(
    //         Commands.parallel(
    //                 new TeleopSwerve(drivetrain, oi::getTranslateX, oi::getTranslateY, () ->
    // (90)),
    //                 Commands.runOnce(
    //                         () -> shooter.setShootingPosition(ShootingPosition.AMP), shooter)
    //                     .withName("prepare to score amp"))
    //             .until(
    //                 () -> {
    //                   return !intake.hasNote();
    //                 }));

    oi.getPrepareToScoreSubwooferButton()
        .onTrue(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.SUBWOOFER), shooter)
                .withName("prepare to score subwoofer"));

    oi.getPrepareToScorePodiumButton()
        .onTrue(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.PODIUM), shooter)
                .withName("prepare to score podium"));

    oi.getStoreShooterButton()
        .onTrue(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.STORAGE), shooter)
                .withName("store shooter"));

    oi.getShootFullFieldButton()
        .toggleOnTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> shooter.setShootingPosition(ShootingPosition.PASS), shooter),
                    new TeleopSwerveAimToPass(
                        drivetrain, shooter, intake, oi::getTranslateX, oi::getTranslateY))
                .withName("prepare to pass"));

    oi.getShooterAngleUpButton()
        .whileTrue(
            Commands.runOnce(
                    () ->
                        shooter.setAngleMotorVoltage(
                            ShooterConstants.ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE),
                    shooter)
                .onlyIf(() -> !shooter.isAutomated())
                .withName("shooter manual up"))
        .onFalse(
            Commands.runOnce(() -> shooter.setAngleMotorVoltage(0), shooter)
                .onlyIf(() -> !shooter.isAutomated())
                .withName("shooter manual up stop"));

    oi.getShooterAngleDownButton()
        .whileTrue(
            Commands.runOnce(
                    () ->
                        shooter.setAngleMotorVoltage(
                            -ShooterConstants.ANGLE_MOTOR_MANUAL_CONTROL_VOLTAGE),
                    shooter)
                .onlyIf(() -> !shooter.isAutomated())
                .withName("shooter manual down"))
        .onFalse(
            Commands.runOnce(() -> shooter.setAngleMotorVoltage(0), shooter)
                .onlyIf(() -> !shooter.isAutomated())
                .withName("shooter manual down stop"));

    oi.getShootButton().whileTrue(getShootCommand());

    oi.getScaleDownShooterVelocityButton()
        .onTrue(Commands.runOnce(shooter::enableScaleDownShooterVelocity));
    oi.getScaleDownShooterVelocityButton()
        .onFalse(Commands.runOnce(shooter::disableScaleDownShooterVelocity));
  }

  private Command getAutoShootNowCommand() {
    return Commands.runOnce(intake::shoot, intake);
  }

  private Command getAutoStopAndShootCommand() {
    return Commands.sequence(
            Commands.runOnce(() -> shooter.setShootingPosition(ShootingPosition.FIELD)),
            Commands.parallel(
                    new TeleopSwerveAimAtSpeaker(drivetrain, shooter, intake, () -> 0.0, () -> 0.0),
                    getShootCommand())
                .withTimeout(1.0))
        .andThen(Commands.runOnce(intake::shoot, intake));
  }

  private Command getShootCommand() {
    return Commands.waitUntil(this::isReadyToShoot)
        .andThen(
            Commands.sequence(
                Commands.runOnce(intake::shoot, intake),
                NoteVisualizer.shoot(),
                Commands.runOnce(drivetrain::disableAimToSpeaker)))
        .withName("shoot");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Check if the alliance color has changed; if so, update the vision subsystem and Field2d
   * singleton.
   */
  public void checkAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() != lastAlliance) {
      this.lastAlliance = alliance.get();
      this.drivetrain.updateAlliance(this.lastAlliance);
      Field2d.getInstance().updateAlliance(this.lastAlliance);
    }
  }

  public void periodic() {
    if (this.isReadyToShoot()) {
      LEDs.getInstance().setShooterLEDState(ShooterLEDState.IS_READY_TO_SHOOT);
    } else if (this.intake.hasNote()) {
      LEDs.getInstance().setShooterLEDState(ShooterLEDState.AIMING_AT_SPEAKER);
    }
  }

  private boolean isReadyToShoot() {
    return shooter.isShooterReadyToShoot(!vision.isEnabled() || drivetrain.isAimedAtSpeaker());
  }
}
