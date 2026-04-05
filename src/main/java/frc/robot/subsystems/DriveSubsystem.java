package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Variables;
import frc.robot.utils.APOdometry;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

public class DriveSubsystem extends SubsystemBase {

  // --- Swerve Modules ---
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // --- Sensors ---
  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGryoID);

  // --- Odometry ---
  private final APOdometry m_odometry;

  public DriveSubsystem() {
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    List<MAXSwerveModule> modules = List.of(
        m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);

    m_odometry = APOdometry.getInstance(modules, m_gyro);
  }

  @Override
  public void periodic() {
    m_odometry.update();
    Pose currentPose = m_odometry.getPose();
    Variables.drive.heading = getHeading();
    Variables.drive.currentX = currentPose.getX();
    Variables.drive.currentY = currentPose.getY();
  }

  // =========================================================
  // POSE / ODOMETRY
  // =========================================================

  /** Normalized pose (0–360°) */
  public Pose getPose() {
    return m_odometry.getPose();
  }

  /** Continuous pose (angle not wrapped) */
  public Pose getPoseContinuous() {
    return m_odometry.getPoseContinuous();
  }

  /** Set robot pose explicitly */
  public void setPose(Pose pose) {
    m_odometry.setPose(pose);
  }

  /** Reset robot to (0,0,current heading) */
  public void resetToOrigin() {
    m_odometry.reset();
  }

  // =========================================================
  // DRIVING
  // =========================================================

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeedDelivered,
                ySpeedDelivered,
                rotDelivered,
                Rotation2d.fromDegrees(getHeading())) // ✅ FIXED
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Direct control of module states (auto/pathing) */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Lock wheels in X formation */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /** Reset all drive encoders */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  // =========================================================
  // GYRO
  // =========================================================

  /** Current heading in degrees */
  public double getHeading() {
    //return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees()).getDegrees();
    double heading = m_gyro.getRotation2d().getDegrees() *
        (DriveConstants.kGyroReversed ? -1.0 : 1.0);

    return Calculations.normalizeAngle360(heading);
  }

  /** Angular velocity (deg/sec) */
  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble() *
        (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Initialize gyro safely, then zero it
   * (used at robot startup)
   */
  public void initializeAndZeroGyro() {

    Timer timer = new Timer();
    timer.start();

    // Wait until gyro gives valid data or timeout
    while (Double.isNaN(m_gyro.getRotation2d().getDegrees()) && timer.get() < 2.0) {
      Timer.delay(0.01);
    }

    m_gyro.setYaw(0);
  }
}