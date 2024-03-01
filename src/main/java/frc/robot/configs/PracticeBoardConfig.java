package frc.robot.configs;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.team3061.RobotConfig;
import frc.lib.team3061.drivetrain.swerve.SwerveConstants;

/*
 * Refer to the README for how to represent your robot's configuration. For more information on
 * these methods, refer to the documentation in the RobotConfig class.
 */
public class PracticeBoardConfig extends RobotConfig {
 

  private static final String CAN_BUS_NAME = "";

  
  @Override
  public boolean getPhoenix6Licensed() {
    return true;
  }

  @Override
  public double getSwerveAngleKP() {
    return 0;
  }

  @Override
  public double getSwerveAngleKI() {
    return 0;
  }

  @Override
  public double getSwerveAngleKD() {
    return 0;
  }

  @Override
  public double getSwerveAngleKS() {
    return 0;
  }

  @Override
  public double getSwerveAngleKV() {
    return 0;
  }

  @Override
  public double getSwerveAngleKA() {
    return 0;
  }

  @Override
  public double getSwerveDriveKP() {
    return 0;
  }

  @Override
  public double getSwerveDriveKI() {
    return 0;
  }

  @Override
  public double getSwerveDriveKD() {
    return 0;
  }

  @Override
  public double getDriveKS() {
    return 0;
  }

  @Override
  public double getDriveKV() {
    return 0;
  }

  @Override
  public double getDriveKA() {
    return 0;
  }

  @Override
  public SwerveConstants getSwerveConstants() {
    return SwerveConstants.MK4I_L3_PLUS_CONSTANTS;
  }

  @Override
  public int[] getSwerveDriveMotorCANIDs() {
    return new int[] {
    };
  }

  @Override
  public int[] getSwerveSteerMotorCANIDs() {
    return new int[] {
      
    };
  }

  @Override
  public int[] getSwerveSteerEncoderCANIDs() {
    return new int[] {
      
    };
  }

  @Override
  public double[] getSwerveSteerOffsets() {
    return new double[] {
      
    };
  }

  @Override
  public int getGyroCANID() {
    return 0;
  }

  @Override
  public double getTrackwidth() {
    return 0;
  }

  @Override
  public double getWheelbase() {
    return 0;
  }

  @Override
  public double getWheelDiameterMeters() {
    return 0;
  }

  @Override
  public double getRobotWidthWithBumpers() {
    return 0;
  }

  @Override
  public double getRobotLengthWithBumpers() {
    return 0;
  }

  @Override
  public Transform3d[] getRobotToCameraTransforms() {
    return new Transform3d[] {
      
    };
  }

  @Override
  public double getRobotMaxVelocity() {
    return 0;
  }

  @Override
  public double getRobotMaxDriveAcceleration() {
    return 0;
  }

  @Override
  public double getRobotMaxTurnAcceleration() {
    return 0;
  }

  @Override
  public double getRobotSlowModeMultiplier() {
    return 0;
  }

  @Override
  public double getRobotMaxCoastVelocity() {
    return 0;
  }

  @Override
  public double getAutoMaxSpeed() {
    return 0;
  }

  @Override
  public double getAutoMaxAcceleration() {
    return 0;
  }

  @Override
  public double getAutoDriveKP() {
    return 0;
  }

  @Override
  public double getAutoDriveKI() {
    return 0;
  }

  @Override
  public double getAutoDriveKD() {
    return 0;
  }

  @Override
  public double getAutoTurnKP() {
    return 0;
  }

  @Override
  public double getAutoTurnKI() {
    return 0;
  }

  @Override
  public double getAutoTurnKD() {
    return 0;
  }

  @Override
  public String getCANBusName() {
    return CAN_BUS_NAME;
  }

  @Override
  public String[] getCameraNames() {
    return new String[] {};
  }

  @Override
  public double getDriveToPoseDriveKP() {
    return 0;
  }

  @Override
  public double getDriveToPoseDriveKD() {
    return 0;
  }

  @Override
  public double getDriveToPoseThetaKI() {
    return 0;
  }

  @Override
  public double getDriveToPoseThetaKP() {
    return 0;
  }

  @Override
  public double getDriveToPoseThetaKD() {
    return 0;
  }

  @Override
  public double getDriveToPoseDriveMaxVelocity() {
    return 0.5;
  }

  @Override
  public double getDriveToPoseDriveMaxAcceleration() {
    return getAutoMaxAcceleration();
  }

  @Override
  public double getDriveToPoseTurnMaxVelocity() {
    return getDriveToPoseDriveMaxVelocity()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  @Override
  public double getDriveToPoseTurnMaxAcceleration() {
    return getDriveToPoseDriveMaxAcceleration()
        / Math.hypot(getTrackwidth() / 2.0, getWheelbase() / 2.0);
  }

  @Override
  public double getDriveToPoseDriveTolerance() {
    return 0;
  }

  @Override
  public double getDriveToPoseThetaTolerance() {
    return 0;
  }

  @Override
  public int getPneumaticsHubCANID() {
    return 0;
  }

  @Override
  public double getMoveToPathFinalVelocity() {
    return 0;
  }

  @Override
  public double getDriveFacingAngleThetaKP() {
    return 0;
  }

  @Override
  public double getDriveFacingAngleThetaKI() {
    return 0;
  }

  @Override
  public double getDriveFacingAngleThetaKD() {
    return 0;
  }

  @Override
  public double getOdometryUpdateFrequency() {
    return 250.0;
  }

  @Override
  public LED_HARDWARE getLEDHardware() {
    return LED_HARDWARE.RIO;
  }

  @Override
  public int getLEDCount() {
    return 0;
  }

  @Override
  public SWERVE_CONTROL_MODE getSwerveSteerControlMode() {
    return SWERVE_CONTROL_MODE.VOLTAGE;
  }

  @Override
  public SWERVE_CONTROL_MODE getSwerveDriveControlMode() {
    return SWERVE_CONTROL_MODE.TORQUE_CURRENT_FOC;
  }
}
