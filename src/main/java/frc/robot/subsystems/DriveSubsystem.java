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
import frc.robot.utils.LimelightToAPOTranslator;
import frc.robot.utils.Pose;
import frc.robot.utils.Vector;

public class DriveSubsystem extends SubsystemBase {

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

  private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.kGryoID);

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

  Variables.drive.currentX = currentPose.getX();
  Variables.drive.currentY = currentPose.getY();
  Variables.drive.heading = getHeading();
  Variables.drive.turnRate = getTurnRate();
}

  public Pose getPose() {
    return m_odometry.getPose();
  }

  public Pose getPoseContinuous() {
    return m_odometry.getPoseContinuous();
  }

  public Vector getFieldVelocity() {
    Vector velocity = m_odometry.getVelocity();
    return new Vector(velocity.getX(), velocity.getY());
  }

  public void setPose(Pose pose) {
    m_odometry.setPose(pose);
  }

  public void resetToOrigin() {
    m_odometry.reset();
  }

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
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);

    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearLeft.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /**
   * Base robot heading used everywhere outside APOdometry.
   * Keep this convention aligned with your real robot tests.
   */
  public double getHeading() {
    double heading = m_gyro.getRotation2d().getDegrees()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);

    return Calculations.normalizeAngle360(heading);
  }

  public double getTurnRate() {
    return m_gyro.getAngularVelocityZWorld().getValueAsDouble()
        * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setGyroYaw(double yawDeg) {
    double rawYaw = DriveConstants.kGyroReversed ? -yawDeg : yawDeg;
    m_gyro.setYaw(rawYaw);
  }

  public void resetGyro() {
    setGyroYaw(0.0);
  }

  public double getVisionYaw() {
  return m_gyro.getRotation2d().getDegrees()
      * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}

public double getVisionYawRate() {
  return m_gyro.getAngularVelocityZWorld().getValueAsDouble()
      * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
}

  /**
   * Hardware init only.
   * Do not force a field heading here.
   */
  public void initializeGyro() {
    Timer timer = new Timer();
    timer.start();

    while (Double.isNaN(m_gyro.getRotation2d().getDegrees()) && timer.get() < 2.0) {
      Timer.delay(0.01); 
    }
  }

  /**
 * Sets odometry pose from Limelight, but rejects large jumps.
 */
/**
 * Applies Limelight position as a translation correction to odometry.
 * Keeps the current drivetrain heading and wheel state intact.
 *
 * This does NOT hard-reset odometry like setPose().
 */
public void correctPositionFromLimelight() {
  if (!Variables.limelight.hasTarget) {
    return;
  }

  Pose visionPose = LimelightToAPOTranslator.getTranslatedPose();
  Pose currentPose = getPose();

  double dx = visionPose.getX() - currentPose.getX();
  double dy = visionPose.getY() - currentPose.getY();

  m_odometry.translatePose(dx, dy);
}
}
