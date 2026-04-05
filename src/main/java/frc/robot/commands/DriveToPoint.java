package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

public class DriveToPoint extends Command {

  // =========================================================
  // Dependencies
  // =========================================================

  private final DriveSubsystem driveSubsystem;

  // =========================================================
  // Control
  // =========================================================

  private final APPID xPID;
  private final APPID yPID;
  private final APPID turnPID;

  private final Pose targetPose;
  private final double positionTolerance;
  private final double angleTolerance;

  // =========================================================
  // Defaults
  // =========================================================

  private static final double kXP = 0.6;
  private static final double kXI = 0.0;
  private static final double kXD = 0.05;
  private static final double kXMaxSpeed = 1.15;

  private static final double kYP = 0.6;
  private static final double kYI = 0.0;
  private static final double kYD = 0.05;
  private static final double kYMaxSpeed = 1.15;

  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;
  private static final double kMaxRotationSpeed = 0.5;

  // =========================================================
  // 🔥 FULLY TUNABLE CONSTRUCTOR
  // =========================================================

  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double xP,
      double yP,
      double turnP,
      double xMaxSpeed,
      double yMaxSpeed,
      double maxTurnSpeed,
      double positionTolerance,
      double angleTolerance) {

    this.driveSubsystem = driveSubsystem;
    this.targetPose = new Pose(targetX, targetY, targetAngle);
    this.positionTolerance = positionTolerance;
    this.angleTolerance = angleTolerance;

    this.xPID = new APPID(xP, kXI, kXD, positionTolerance);
    this.xPID.setMaxOutput(xMaxSpeed);

    this.yPID = new APPID(yP, kYI, kYD, positionTolerance);
    this.yPID.setMaxOutput(yMaxSpeed);

    this.turnPID = new APPID(turnP, kTurnI, kTurnD, angleTolerance);
    this.turnPID.setMaxOutput(maxTurnSpeed);

    addRequirements(driveSubsystem);
  }

  // =========================================================
  // Constructor (tunable without tolerances)
  // =========================================================

  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double xP,
      double yP,
      double turnP,
      double xMaxSpeed,
      double yMaxSpeed,
      double maxTurnSpeed) {

    this(
        driveSubsystem,
        targetX, targetY, targetAngle,
        xP, yP, turnP,
        xMaxSpeed, yMaxSpeed, maxTurnSpeed,
        0.1, 2.0);
  }

  // =========================================================
  // Constructor (original tolerances)
  // =========================================================

  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double positionTolerance,
      double angleTolerance) {

    this(
        driveSubsystem,
        targetX, targetY, targetAngle,
        kXP, kYP, kTurnP,
        kXMaxSpeed, kYMaxSpeed, kMaxRotationSpeed,
        positionTolerance, angleTolerance);
  }

  // =========================================================
  // Constructor (default everything)
  // =========================================================

  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle) {

    this(
        driveSubsystem,
        targetX, targetY, targetAngle,
        kXP, kYP, kTurnP,
        kXMaxSpeed, kYMaxSpeed, kMaxRotationSpeed,
        0.1, 2.0);
  }

  // =========================================================
  // Lifecycle
  // =========================================================

  @Override
  public void initialize() {
    xPID.reset();
    yPID.reset();
    turnPID.reset();

    SmartDashboard.putNumber("TARGET_X", targetPose.getX());
    SmartDashboard.putNumber("TARGET_Y", targetPose.getY());
    SmartDashboard.putNumber("TARGET_ANGLE", targetPose.getAngle());
  }

  @Override
  public void execute() {

    Pose currentPose = driveSubsystem.getPose();

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    xPID.setDesiredValue(0);
    yPID.setDesiredValue(0);

    double xSpeed = xPID.calculate(-xError);
    double ySpeed = yPID.calculate(-yError);

    // Rotation
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());

    double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);

    turnPID.setDesiredValue(0);
    double rotationSpeed = turnPID.calculate(-angleError);

    driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {

    driveSubsystem.drive(0, 0, 0, true);

    if (!interrupted) {
      driveSubsystem.setPose(new Pose(
          targetPose.getX(),
          targetPose.getY(),
          targetPose.getAngle()));
    }
  }

  @Override
  public boolean isFinished() {

    Pose currentPose = driveSubsystem.getPose();

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double distance = Math.hypot(xError, yError);

    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());
    double angleError = Math.abs(
        Calculations.shortestAngularDistance(targetAngle, currentAngle));

    return distance <= positionTolerance && angleError <= angleTolerance;
  }
}