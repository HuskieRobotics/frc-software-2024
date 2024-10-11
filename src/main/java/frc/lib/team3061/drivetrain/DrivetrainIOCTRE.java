package frc.lib.team3061.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.DeviceEnableValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.Conversions;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;
import frc.lib.team3061.gyro.GyroIO.GyroIOInputs;
import frc.lib.team3061.util.RobotOdometry;
import frc.lib.team6328.util.TunableNumber;
import frc.robot.Constants;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.Logger;

public class DrivetrainIOCTRE extends SwerveDrivetrain implements DrivetrainIO {

  static class CustomSlotGains extends Slot0Configs {
    public CustomSlotGains(double kP, double kI, double kD, double kA, double kV, double kS) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.kA = kA;
      this.kV = kV;
      this.kS = kS;
    }
  }

  static class SwerveModuleSignals {
    public SwerveModuleSignals(TalonFX driveMotor, TalonFX steerMotor) {
      this.steerVelocityStatusSignal = steerMotor.getVelocity().clone();
      this.steerAccelerationStatusSignal = steerMotor.getAcceleration().clone();
      this.steerPositionErrorStatusSignal = steerMotor.getClosedLoopError().clone();
      this.steerPositionReferenceStatusSignal = steerMotor.getClosedLoopReference().clone();
      this.drivePositionStatusSignal = driveMotor.getPosition().clone();
      this.driveVelocityErrorStatusSignal = driveMotor.getClosedLoopError().clone();
      this.driveVelocityReferenceStatusSignal = driveMotor.getClosedLoopReference().clone();
      this.driveAccelerationStatusSignal = driveMotor.getAcceleration().clone();
    }

    StatusSignal<Double> steerVelocityStatusSignal;
    StatusSignal<Double> steerAccelerationStatusSignal;
    StatusSignal<Double> steerPositionErrorStatusSignal;
    StatusSignal<Double> steerPositionReferenceStatusSignal;
    StatusSignal<Double> drivePositionStatusSignal;
    StatusSignal<Double> driveVelocityErrorStatusSignal;
    StatusSignal<Double> driveVelocityReferenceStatusSignal;
    StatusSignal<Double> driveAccelerationStatusSignal;
  }

  private final TunableNumber driveKp =
      new TunableNumber("Drivetrain/DriveKp", RobotConfig.getInstance().getSwerveDriveKP());
  private final TunableNumber driveKi =
      new TunableNumber("Drivetrain/DriveKi", RobotConfig.getInstance().getSwerveDriveKI());
  private final TunableNumber driveKd =
      new TunableNumber("Drivetrain/DriveKd", RobotConfig.getInstance().getSwerveDriveKD());
  private final TunableNumber steerKp =
      new TunableNumber("Drivetrain/TurnKp", RobotConfig.getInstance().getSwerveAngleKP());
  private final TunableNumber steerKi =
      new TunableNumber("Drivetrain/TurnKi", RobotConfig.getInstance().getSwerveAngleKI());
  private final TunableNumber steerKd =
      new TunableNumber("Drivetrain/TurnKd", RobotConfig.getInstance().getSwerveAngleKD());

  protected static final TunableNumber driveFacingAngleThetaKp =
      new TunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKp",
          RobotConfig.getInstance().getDriveFacingAngleThetaKP());
  protected static final TunableNumber driveFacingAngleThetaKi =
      new TunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKi",
          RobotConfig.getInstance().getDriveFacingAngleThetaKI());
  protected static final TunableNumber driveFacingAngleThetaKd =
      new TunableNumber(
          "Drivetrain/DriveFacingAngle/ThetaKd",
          RobotConfig.getInstance().getDriveFacingAngleThetaKD());

  private static final CustomSlotGains steerGains =
      new CustomSlotGains(
          RobotConfig.getInstance().getSwerveAngleKP(),
          RobotConfig.getInstance().getSwerveAngleKI(),
          RobotConfig.getInstance().getSwerveAngleKD(),
          RobotConfig.getInstance().getSwerveAngleKA(),
          RobotConfig.getInstance().getSwerveAngleKV(),
          RobotConfig.getInstance().getSwerveAngleKS());
  private static final CustomSlotGains driveGains =
      new CustomSlotGains(
          RobotConfig.getInstance().getSwerveDriveKP(),
          RobotConfig.getInstance().getSwerveDriveKI(),
          RobotConfig.getInstance().getSwerveDriveKD(),
          RobotConfig.getInstance().getDriveKA(),
          RobotConfig.getInstance().getDriveKV(),
          RobotConfig.getInstance().getDriveKS());

  // The closed-loop output type to use for the steer motors
  // This affects the PID/FF gains for the steer motors
  // TorqueCurrentFOC is not currently supported in simulation.
  private static final ClosedLoopOutputType steerClosedLoopOutput = getSteerClosedLoopOutputType();

  // The closed-loop output type to use for the drive motors
  // This affects the PID/FF gains for the drive motors
  // TorqueCurrentFOC is not currently supported in simulation.
  private static final ClosedLoopOutputType driveClosedLoopOutput = getDriveClosedLoopOutputType();

  private static final double COUPLE_RATIO =
      RobotConfig.getInstance().getAzimuthSteerCouplingRatio();
  private static final double STEER_INERTIA = 0.00001;
  private static final double DRIVE_INERTIA = 0.001;

  private static final SwerveDrivetrainConstants drivetrainConstants =
      new SwerveDrivetrainConstants()
          .withPigeon2Id(RobotConfig.getInstance().getGyroCANID())
          .withCANbusName(RobotConfig.getInstance().getCANBusName())
          .withPigeon2Configs(
              new Pigeon2Configuration()
                  .withMountPose(new MountPoseConfigs().withMountPoseRoll(-180.0)));

  private static final SwerveModuleConstantsFactory constantCreator =
      new SwerveModuleConstantsFactory()
          .withDriveMotorInitialConfigs(
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(SwerveConstants.DRIVE_CONTINUOUS_CURRENT_LIMIT)
                          .withSupplyCurrentThreshold(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
                          .withSupplyTimeThreshold(SwerveConstants.DRIVE_PEAK_CURRENT_DURATION)
                          .withSupplyCurrentLimitEnable(
                              SwerveConstants.DRIVE_ENABLE_CURRENT_LIMIT)))
          .withSteerMotorInitialConfigs(
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withSupplyCurrentLimit(SwerveConstants.ANGLE_CONTINUOUS_CURRENT_LIMIT)
                          .withSupplyCurrentThreshold(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                          .withSupplyTimeThreshold(SwerveConstants.ANGLE_PEAK_CURRENT_DURATION)
                          .withSupplyCurrentLimitEnable(SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)
                          .withStatorCurrentLimit(SwerveConstants.ANGLE_PEAK_CURRENT_LIMIT)
                          .withStatorCurrentLimitEnable(
                              SwerveConstants.ANGLE_ENABLE_CURRENT_LIMIT)))
          .withDriveMotorGearRatio(
              RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio())
          .withSteerMotorGearRatio(
              RobotConfig.getInstance().getSwerveConstants().getAngleGearRatio())
          .withWheelRadius(
              Units.metersToInches(RobotConfig.getInstance().getWheelDiameterMeters() / 2.0))
          .withSlipCurrent(SwerveConstants.DRIVE_PEAK_CURRENT_LIMIT)
          .withSteerMotorGains(steerGains)
          .withDriveMotorGains(driveGains)
          .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
          .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
          .withSpeedAt12VoltsMps(RobotConfig.getInstance().getRobotMaxVelocity())
          .withSteerInertia(STEER_INERTIA)
          .withDriveInertia(DRIVE_INERTIA)
          .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
          .withCouplingGearRatio(
              COUPLE_RATIO) // Every 1 rotation of the azimuth results in couple ratio drive turns
          .withSteerMotorInverted(
              RobotConfig.getInstance().getSwerveConstants().isAngleMotorInverted());

  private static final SwerveModuleConstants frontLeft =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[0],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[0],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[0],
          RobotConfig.getInstance().getSwerveSteerOffsets()[0],
          RobotConfig.getInstance().getWheelbase() / 2.0,
          RobotConfig.getInstance().getTrackwidth() / 2.0,
          !RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted());
  private static final SwerveModuleConstants frontRight =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[1],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[1],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[1],
          RobotConfig.getInstance().getSwerveSteerOffsets()[1],
          RobotConfig.getInstance().getWheelbase() / 2.0,
          -RobotConfig.getInstance().getTrackwidth() / 2.0,
          RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted());
  private static final SwerveModuleConstants backLeft =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[2],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[2],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[2],
          RobotConfig.getInstance().getSwerveSteerOffsets()[2],
          -RobotConfig.getInstance().getWheelbase() / 2.0,
          RobotConfig.getInstance().getTrackwidth() / 2.0,
          !RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted());
  private static final SwerveModuleConstants backRight =
      constantCreator.createModuleConstants(
          RobotConfig.getInstance().getSwerveSteerMotorCANIDs()[3],
          RobotConfig.getInstance().getSwerveDriveMotorCANIDs()[3],
          RobotConfig.getInstance().getSwerveSteerEncoderCANIDs()[3],
          RobotConfig.getInstance().getSwerveSteerOffsets()[3],
          -RobotConfig.getInstance().getWheelbase() / 2.0,
          -RobotConfig.getInstance().getTrackwidth() / 2.0,
          RobotConfig.getInstance().getSwerveConstants().isDriveMotorInverted());

  // gyro signals
  private final StatusSignal<Double> pitchStatusSignal;
  private final StatusSignal<Double> rollStatusSignal;
  private final StatusSignal<Double> angularVelocityXStatusSignal;
  private final StatusSignal<Double> angularVelocityYStatusSignal;

  // swerve module signals
  SwerveModuleSignals[] swerveModulesSignals = new SwerveModuleSignals[4];

  private Translation2d centerOfRotation;
  private ChassisSpeeds targetChassisSpeeds;

  private SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();
  private SwerveRequest.RobotCentric driveRobotCentricRequest = new SwerveRequest.RobotCentric();
  private SwerveRequest.FieldCentric driveFieldCentricRequest = new SwerveRequest.FieldCentric();
  private SwerveRequest.FieldCentricFacingAngle driveFacingAngleRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();
  private SwerveRequest.PointWheelsAt pointRequest = new SwerveRequest.PointWheelsAt();
  private SwerveRequest.ApplyChassisSpeeds applyChassisSpeedsRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  // only used for TorqueCurrentFOC characterization
  private TorqueCurrentFOC[] driveCurrentRequests = new TorqueCurrentFOC[4];
  private TorqueCurrentFOC[] steerCurrentRequests = new TorqueCurrentFOC[4];

  // queues for odometry updates from CTRE's thread

  private final Lock odometryLock = new ReentrantLock();
  private final List<Queue<Double>> drivePositionQueues = new ArrayList<>();
  private final List<Queue<Double>> steerPositionQueues = new ArrayList<>();
  private final Queue<Double> gyroYawQueue;
  private final Queue<Double> timestampQueue = new ArrayBlockingQueue<>(20);
  private final PhoenixOdometryThread odometryThread = new PhoenixOdometryThread();

  /**
   * Creates a new Drivetrain subsystem.
   *
   * @param gyroIO the abstracted interface for the gyro for the drivetrain
   * @param flModule the front left swerve module
   * @param frModule the front right swerve module
   * @param blModule the back left swerve module
   * @param brModule the back right swerve module
   */
  public DrivetrainIOCTRE() {
    super(drivetrainConstants, 250.0, frontLeft, frontRight, backLeft, backRight);

    this.pitchStatusSignal = this.m_pigeon2.getPitch().clone();
    this.pitchStatusSignal.setUpdateFrequency(100);
    this.rollStatusSignal = this.m_pigeon2.getRoll().clone();
    this.rollStatusSignal.setUpdateFrequency(100);
    this.angularVelocityXStatusSignal = this.m_pigeon2.getAngularVelocityXWorld().clone();
    this.angularVelocityXStatusSignal.setUpdateFrequency(100);
    this.angularVelocityYStatusSignal = this.m_pigeon2.getAngularVelocityYWorld().clone();
    this.angularVelocityYStatusSignal.setUpdateFrequency(100);

    for (int i = 0; i < swerveModulesSignals.length; i++) {
      swerveModulesSignals[i] =
          new SwerveModuleSignals(this.Modules[i].getSteerMotor(), this.Modules[i].getDriveMotor());
    }

    this.centerOfRotation = new Translation2d(); // default to (0,0)

    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    for (int i = 0; i < driveCurrentRequests.length; i++) {
      this.driveCurrentRequests[i] = new TorqueCurrentFOC(0.0);
      this.steerCurrentRequests[i] = new TorqueCurrentFOC(0.0);
    }

    // configure PID for drive facing angle
    this.driveFacingAngleRequest.HeadingController.setPID(
        driveFacingAngleThetaKp.get(),
        driveFacingAngleThetaKi.get(),
        driveFacingAngleThetaKd.get());
    this.driveFacingAngleRequest.HeadingController.enableContinuousInput(0, Math.PI * 2);

    // always define 0° (towards the red alliance) as "forward"; the Drivetrain subsystem handles
    //  the definition of forward based on the current alliance
    this.driveFacingAngleRequest.ForwardReference = SwerveRequest.ForwardReference.RedAlliance;
    this.driveFieldCentricRequest.ForwardReference = SwerveRequest.ForwardReference.RedAlliance;

    // create queues for updates from our odometry thread
    for (int i = 0; i < this.Modules.length; i++) {
      this.drivePositionQueues.add(
          this.odometryThread.registerSignal(
              this.Modules[i].getDriveMotor(), this.Modules[i].getDriveMotor().getPosition()));
      this.steerPositionQueues.add(
          this.odometryThread.registerSignal(
              this.Modules[i].getSteerMotor(), this.Modules[i].getSteerMotor().getPosition()));
    }
    this.gyroYawQueue = this.odometryThread.registerSignal(this.m_pigeon2, this.m_pigeon2.getYaw());
  }

  @Override
  public void updateInputs(DrivetrainIOInputsCollection inputs) {

    // update and log gyro inputs
    this.updateGyroInputs(inputs.gyro);

    // update and log the swerve modules inputs
    for (int i = 0; i < swerveModulesSignals.length; i++) {
      this.updateSwerveModuleInputs(inputs.swerve[i], this.Modules[i], swerveModulesSignals[i]);
    }

    inputs.drivetrain.swerveMeasuredStates = this.getState().ModuleStates;
    inputs.drivetrain.swerveReferenceStates = this.getState().ModuleTargets;

    inputs.drivetrain.targetVXMetersPerSec = this.targetChassisSpeeds.vxMetersPerSecond;
    inputs.drivetrain.targetVYMetersPerSec = this.targetChassisSpeeds.vyMetersPerSecond;
    inputs.drivetrain.targetAngularVelocityRadPerSec =
        this.targetChassisSpeeds.omegaRadiansPerSecond;

    ChassisSpeeds measuredChassisSpeeds =
        m_kinematics.toChassisSpeeds(this.getState().ModuleStates);
    inputs.drivetrain.measuredVXMetersPerSec = measuredChassisSpeeds.vxMetersPerSecond;
    inputs.drivetrain.measuredVYMetersPerSec = measuredChassisSpeeds.vyMetersPerSecond;
    inputs.drivetrain.measuredAngularVelocityRadPerSec =
        measuredChassisSpeeds.omegaRadiansPerSecond;

    inputs.drivetrain.averageDriveCurrent = this.getAverageDriveCurrent(inputs);

    this.odometryLock.lock();

    inputs.drivetrain.odometryTimestamps =
        this.timestampQueue.stream().mapToDouble(Double::valueOf).toArray();
    this.timestampQueue.clear();

    inputs.gyro.odometryYawPositions =
        this.gyroYawQueue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    this.gyroYawQueue.clear();

    for (int i = 0; i < swerveModulesSignals.length; i++) {
      inputs.swerve[i].odometryDrivePositionsMeters =
          drivePositionQueues.get(i).stream()
              .mapToDouble(
                  signalValue ->
                      Conversions.falconRotationsToMechanismMeters(
                          signalValue,
                          RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
                          RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio()))
              .toArray();
      inputs.swerve[i].odometryTurnPositions =
          steerPositionQueues.get(i).stream()
              .map(Rotation2d::fromRotations)
              .toArray(Rotation2d[]::new);
      drivePositionQueues.get(i).clear();
      steerPositionQueues.get(i).clear();
    }

    this.odometryLock.unlock();

    if (Constants.getMode() == Constants.Mode.SIM) {
      updateSimState(Constants.LOOP_PERIOD_SECS, 12.0);
    }

    // update tunables
    if (driveKp.hasChanged()
        || driveKi.hasChanged()
        || driveKd.hasChanged()
        || steerKp.hasChanged()
        || steerKi.hasChanged()
        || steerKd.hasChanged()) {
      for (SwerveModule swerveModule : this.Modules) {
        Slot0Configs driveSlot0 = new Slot0Configs();
        swerveModule.getDriveMotor().getConfigurator().refresh(driveSlot0);
        driveSlot0.kP = driveKp.get();
        driveSlot0.kI = driveKi.get();
        driveSlot0.kD = driveKd.get();
        swerveModule.getDriveMotor().getConfigurator().apply(driveSlot0);

        Slot0Configs steerSlot0 = new Slot0Configs();
        swerveModule.getSteerMotor().getConfigurator().refresh(steerSlot0);
        steerSlot0.kP = steerKp.get();
        steerSlot0.kI = steerKi.get();
        steerSlot0.kD = steerKd.get();
        swerveModule.getSteerMotor().getConfigurator().apply(steerSlot0);
      }
    }

    if (driveFacingAngleThetaKp.hasChanged()
        || driveFacingAngleThetaKi.hasChanged()
        || driveFacingAngleThetaKd.hasChanged()) {
      this.driveFacingAngleRequest.HeadingController.setPID(
          driveFacingAngleThetaKp.get(),
          driveFacingAngleThetaKi.get(),
          driveFacingAngleThetaKd.get());
    }
  }

  private void updateGyroInputs(GyroIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        this.pitchStatusSignal,
        this.rollStatusSignal,
        this.angularVelocityXStatusSignal,
        this.angularVelocityYStatusSignal);

    inputs.connected = (this.m_yawGetter.getStatus() == StatusCode.OK);
    inputs.yawDeg =
        BaseStatusSignal.getLatencyCompensatedValue(this.m_yawGetter, this.m_angularVelocity);
    inputs.pitchDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.pitchStatusSignal, this.angularVelocityYStatusSignal);
    inputs.rollDeg =
        BaseStatusSignal.getLatencyCompensatedValue(
            this.rollStatusSignal, this.angularVelocityXStatusSignal);
    inputs.rollDegPerSec = this.angularVelocityXStatusSignal.getValue();
    inputs.pitchDegPerSec = this.angularVelocityYStatusSignal.getValue();
    inputs.yawDegPerSec = this.m_angularVelocity.getValue();
  }

  private void updateSwerveModuleInputs(
      SwerveIOInputs inputs, SwerveModule module, SwerveModuleSignals signals) {

    BaseStatusSignal.refreshAll(
        signals.steerVelocityStatusSignal,
        signals.steerAccelerationStatusSignal,
        signals.steerPositionErrorStatusSignal,
        signals.steerPositionReferenceStatusSignal,
        signals.drivePositionStatusSignal,
        signals.driveVelocityErrorStatusSignal,
        signals.driveVelocityReferenceStatusSignal,
        signals.driveAccelerationStatusSignal);

    SwerveModulePosition position = module.getPosition(false);
    SwerveModuleState state = module.getCurrentState();

    inputs.driveEnabled =
        module.getDriveMotor().getDeviceEnable().getValue() == DeviceEnableValue.Enabled;
    inputs.driveDistanceMeters = position.distanceMeters;
    inputs.driveVelocityMetersPerSec = state.speedMetersPerSecond;

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.driveVelocityReferenceMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            module.getDriveMotor().getClosedLoopReference().getValueAsDouble(),
            RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
            RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio());
    inputs.driveVelocityErrorMetersPerSec =
        Conversions.falconRPSToMechanismMPS(
            module.getDriveMotor().getClosedLoopError().getValueAsDouble(),
            RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
            RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio());
    inputs.driveAccelerationMetersPerSecPerSec =
        Conversions.falconRPSToMechanismMPS(
            signals.driveAccelerationStatusSignal.getValue(),
            RobotConfig.getInstance().getWheelDiameterMeters() * Math.PI,
            RobotConfig.getInstance().getSwerveConstants().getDriveGearRatio());
    inputs.driveAppliedVolts = module.getDriveMotor().getMotorVoltage().getValue();
    inputs.driveStatorCurrentAmps = module.getDriveMotor().getStatorCurrent().getValue();
    inputs.driveSupplyCurrentAmps = module.getDriveMotor().getSupplyCurrent().getValue();
    inputs.driveTempCelsius = module.getDriveMotor().getDeviceTemp().getValue();

    inputs.steerAbsolutePositionDeg = module.getCANcoder().getAbsolutePosition().getValue() * 360.0;

    inputs.steerEnabled =
        module.getSteerMotor().getDeviceEnable().getValue() == DeviceEnableValue.Enabled;
    // since we are using the FusedCANcoder feature, the position and velocity signal for the angle
    // motor accounts for the gear ratio; so, pass a gear ratio of 1 to just convert from rotations
    // to degrees.
    inputs.steerPositionDeg = position.angle.getDegrees();

    // Retrieve the closed loop reference status signals directly from the motor in this method
    // instead of retrieving in advance because the status signal returned depends on the current
    // control mode.
    inputs.steerPositionReferenceDeg =
        Conversions.falconRotationsToMechanismDegrees(
            module.getSteerMotor().getClosedLoopReference().getValueAsDouble(), 1);
    inputs.steerPositionErrorDeg =
        Conversions.falconRotationsToMechanismDegrees(
            module.getSteerMotor().getClosedLoopError().getValueAsDouble(), 1);
    inputs.steerVelocityRevPerMin =
        Conversions.falconRPSToMechanismRPM(signals.steerVelocityStatusSignal.getValue(), 1);
    inputs.steerAccelerationMetersPerSecPerSec =
        Conversions.falconRPSToMechanismRPM(signals.steerAccelerationStatusSignal.getValue(), 1);

    inputs.steerAppliedVolts = module.getSteerMotor().getMotorVoltage().getValue();
    inputs.steerStatorCurrentAmps = module.getSteerMotor().getStatorCurrent().getValue();
    inputs.steerSupplyCurrentAmps = module.getSteerMotor().getSupplyCurrent().getValue();
    inputs.steerTempCelsius = module.getSteerMotor().getDeviceTemp().getValue();
  }

  @Override
  public void holdXStance() {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    this.setControl(this.brakeRequest);
  }

  @Override
  public void driveFieldRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {

    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, rotationalVelocity, this.getState().Pose.getRotation()),
            Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveFieldCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    } else {
      this.setControl(
          this.driveFieldCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    }
  }

  @Override
  public void driveFieldRelativeFacingAngle(
      double xVelocity, double yVelocity, Rotation2d targetDirection, boolean isOpenLoop) {
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, 0.0, getState().Pose.getRotation()),
            Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveFacingAngleRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withTargetDirection(targetDirection));
    } else {
      this.setControl(
          this.driveFacingAngleRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withTargetDirection(targetDirection));
    }
  }

  @Override
  public void pointWheelsAt(Rotation2d targetDirection) {
    this.targetChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    this.setControl(this.pointRequest.withModuleDirection(targetDirection));
  }

  @Override
  public void driveRobotRelative(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean isOpenLoop) {
    this.targetChassisSpeeds =
        ChassisSpeeds.discretize(
            new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity),
            Constants.LOOP_PERIOD_SECS);

    if (isOpenLoop) {
      this.setControl(
          this.driveRobotCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    } else {
      this.setControl(
          this.driveRobotCentricRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withVelocityX(xVelocity)
              .withVelocityY(yVelocity)
              .withRotationalRate(rotationalVelocity));
    }
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds, boolean isOpenLoop) {
    this.targetChassisSpeeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    this.targetChassisSpeeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
    this.targetChassisSpeeds.vyMetersPerSecond = speeds.vyMetersPerSecond;

    if (isOpenLoop) {
      this.setControl(
          this.applyChassisSpeedsRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withSpeeds(speeds)
              .withCenterOfRotation(this.centerOfRotation));
    } else {
      this.setControl(
          this.applyChassisSpeedsRequest
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
              .withSteerRequestType(SwerveModule.SteerRequestType.MotionMagicExpo)
              .withSpeeds(speeds)
              .withCenterOfRotation(this.centerOfRotation));
    }
  }

  @Override
  public void setGyroOffset(double expectedYaw) {
    this.seedFieldRelative(
        new Pose2d(
            RobotOdometry.getInstance().getEstimatedPosition().getTranslation(),
            Rotation2d.fromDegrees(expectedYaw)));
  }

  @Override
  public void setCenterOfRotation(Translation2d centerOfRotation) {
    this.centerOfRotation = centerOfRotation;
  }

  @Override
  public void resetPose(Pose2d pose) {
    this.seedFieldRelative(pose);
  }

  @Override
  public void setDriveMotorCurrent(double amps) {
    // ensure that the SwerveDrivetrain class doesn't control either motor
    this.setControl(idleRequest);

    for (int i = 0; i < this.Modules.length; i++) {
      this.Modules[i].getDriveMotor().setControl(driveCurrentRequests[i].withOutput(amps));
    }
  }

  @Override
  public void setSteerMotorCurrent(double amps) {
    // ensure that the SwerveDrivetrain class doesn't control either motor
    this.setControl(idleRequest);

    for (int i = 0; i < this.Modules.length; i++) {
      this.Modules[i].getSteerMotor().setControl(steerCurrentRequests[i].withOutput(amps));
    }
  }

  @Override
  public void setBrakeMode(boolean enable) {
    for (SwerveModule swerveModule : this.Modules) {
      MotorOutputConfigs config = new MotorOutputConfigs();
      swerveModule.getDriveMotor().getConfigurator().refresh(config);
      config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
      swerveModule.getDriveMotor().getConfigurator().apply(config);
    }
  }

  /**
   * Returns the average current of the swerve module drive motors in amps.
   *
   * @return the average current of the swerve module drive motors in amps
   */
  private double getAverageDriveCurrent(DrivetrainIOInputsCollection inputs) {
    double totalCurrent = 0.0;
    for (SwerveIOInputs swerveInputs : inputs.swerve) {
      totalCurrent += swerveInputs.driveStatorCurrentAmps;
    }
    return totalCurrent / inputs.swerve.length;
  }

  private static ClosedLoopOutputType getSteerClosedLoopOutputType() {
    if (RobotConfig.getInstance().getSwerveSteerControlMode()
        == RobotConfig.SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC) {
      return ClosedLoopOutputType.TorqueCurrentFOC;
    } else {
      return ClosedLoopOutputType.Voltage;
    }
  }

  private static ClosedLoopOutputType getDriveClosedLoopOutputType() {
    if (RobotConfig.getInstance().getSwerveDriveControlMode()
        == RobotConfig.SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC) {
      return ClosedLoopOutputType.TorqueCurrentFOC;
    } else {
      return ClosedLoopOutputType.Voltage;
    }
  }

  // Heavily based on 6328's PhoenixOdometryThread class from 2024.

  /**
   * Provides an interface for asynchronously reading high-frequency measurements to a set of
   * queues.
   *
   * <p>This version is intended for Phoenix 6 devices on both the RIO and CANivore buses. When
   * using a CANivore, the thread uses the "waitForAll" blocking method to enable more consistent
   * sampling. This also allows Phoenix Pro users to benefit from lower latency between devices
   * using CANivore time synchronization.
   */
  private class PhoenixOdometryThread extends Thread {
    private final Lock signalsLock =
        new ReentrantLock(); // Prevents conflicts when registering signals
    private BaseStatusSignal[] signals = new BaseStatusSignal[0];
    private final List<Queue<Double>> queues = new ArrayList<>();
    private boolean isCANFD = false;

    private PhoenixOdometryThread() {
      setName("PhoenixOdometryThread");
      setDaemon(true);
      start();
    }

    public Queue<Double> registerSignal(ParentDevice device, StatusSignal<Double> signal) {
      Queue<Double> queue = new ArrayBlockingQueue<>(20);
      signalsLock.lock();
      odometryLock.lock();
      try {
        isCANFD = CANBus.isNetworkFD(device.getNetwork());
        BaseStatusSignal[] newSignals = new BaseStatusSignal[signals.length + 1];
        System.arraycopy(signals, 0, newSignals, 0, signals.length);
        newSignals[signals.length] = signal.clone();
        BaseStatusSignal.setUpdateFrequencyForAll(
            RobotConfig.getInstance().getOdometryUpdateFrequency(), newSignals[signals.length]);
        signals = newSignals;
        queues.add(queue);
      } finally {
        signalsLock.unlock();
        odometryLock.unlock();
      }
      return queue;
    }

    @Override
    public void run() {
      while (true) {
        // Wait for updates from all signals
        signalsLock.lock();
        try {
          if (isCANFD) {
            BaseStatusSignal.waitForAll(Constants.LOOP_PERIOD_SECS, signals);
          } else {
            Thread.sleep((long) (1000.0 / RobotConfig.getInstance().getOdometryUpdateFrequency()));
            if (signals.length > 0) BaseStatusSignal.refreshAll(signals);
          }
        } catch (InterruptedException e) {
          e.printStackTrace();
        } finally {
          signalsLock.unlock();
        }
        double fpgaTimestamp = Logger.getRealTimestamp() / 1.0e6;

        // Save new data to queues
        odometryLock.lock();
        try {
          for (int i = 0; i < signals.length; i++) {
            queues.get(i).offer(signals[i].getValueAsDouble());
          }
          timestampQueue.offer(fpgaTimestamp);
        } finally {
          odometryLock.unlock();
        }
      }
    }
  }
}
