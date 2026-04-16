// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

/**
 * Command to drive the robot to a specific field position and orientation.
 *
 * Supports both:
 * - precise stop points
 * - continuous pass-through waypoints
 */
public class DriveToPoint extends Command {

  // ===========================================================================================
  // Dependencies
  // ===========================================================================================

  private final DriveSubsystem driveSubsystem;

  // ===========================================================================================
  // Control Parameters
  // ===========================================================================================

  private final APPID xPID;
  private final APPID yPID;
  private final APPID turnPID;

  private final Pose targetPose;
  private final double positionTolerance;
  private final double angleTolerance;
  private final boolean isContinuous;

  // ===========================================================================================
  // PID Constants - X Controller
  // ===========================================================================================

  private static final double kXP = 0.8;
  private static final double kXI = 0.0;
  private static final double kXD = 0.05;
  private static final double kXMaxSpeed = 0.65;

  // ===========================================================================================
  // PID Constants - Y Controller
  // ===========================================================================================

  private static final double kYP = 0.8;
  private static final double kYI = 0.0;
  private static final double kYD = 0.05;
  private static final double kYMaxSpeed = 0.65;

  // ===========================================================================================
  // PID Constants - Rotation
  // ===========================================================================================

  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.001;
  private static final double kMaxRotationSpeed = 0.2;

  // ===========================================================================================
  // Constructors
  // ===========================================================================================

  /**
   * Full constructor.
   *
   * @param driveSubsystem drive subsystem
   * @param targetX target x position (meters)
   * @param targetY target y position (meters)
   * @param targetAngle target heading (degrees)
   * @param positionTolerance acceptable position error (meters)
   * @param angleTolerance acceptable angle error (degrees)
   * @param isContinuous true if this is a pass-through waypoint
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double positionTolerance,
      double angleTolerance,
      boolean isContinuous) {
    this(
        driveSubsystem,
        targetX,
        targetY,
        targetAngle,
        positionTolerance,
        angleTolerance,
        isContinuous,
        kXP,
        kXMaxSpeed,
        kYP,
        kYMaxSpeed);
  }

  /**
   * Full constructor with per-command translation PID and speed tuning.
   *
   * @param driveSubsystem drive subsystem
   * @param targetX target x position (meters)
   * @param targetY target y position (meters)
   * @param targetAngle target heading (degrees)
   * @param positionTolerance acceptable position error (meters)
   * @param angleTolerance acceptable angle error (degrees)
   * @param isContinuous true if this is a pass-through waypoint
   * @param xP x-axis proportional gain
   * @param xMaxSpeed x-axis maximum output/speed
   * @param yP y-axis proportional gain
   * @param yMaxSpeed y-axis maximum output/speed
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double positionTolerance,
      double angleTolerance,
      boolean isContinuous,
      double xP,
      double xMaxSpeed,
      double yP,
      double yMaxSpeed) {
    this.driveSubsystem = driveSubsystem;
    this.targetPose = new Pose(targetX, targetY, targetAngle);
    this.isContinuous = isContinuous;

    // Looser tolerances for continuous points
    if (isContinuous) {
      this.positionTolerance = positionTolerance * 1.625;
      this.angleTolerance = angleTolerance * 1.05;
    } else {
      this.positionTolerance = positionTolerance;
      this.angleTolerance = angleTolerance;
    }

    this.xPID = new APPID(xP, kXI, kXD, this.positionTolerance);
    this.xPID.setMaxOutput(xMaxSpeed);

    this.yPID = new APPID(yP, kYI, kYD, this.positionTolerance);
    this.yPID.setMaxOutput(yMaxSpeed);

    this.turnPID = new APPID(kTurnP, kTurnI, kTurnD, this.angleTolerance);
    this.turnPID.setMaxOutput(kMaxRotationSpeed);

    addRequirements(driveSubsystem);
  }

  /**
   * Constructor with default tolerances and per-command translation PID/speed tuning.
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      boolean isContinuous,
      double xP,
      double xMaxSpeed,
      double yP,
      double yMaxSpeed) {
    this(
        driveSubsystem,
        targetX,
        targetY,
        targetAngle,
        0.1,
        2.0,
        isContinuous,
        xP,
        xMaxSpeed,
        yP,
        yMaxSpeed);
  }

  /**
   * Precise-stop constructor with custom tolerances and per-command translation PID/speed tuning.
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double positionTolerance,
      double angleTolerance,
      double xP,
      double xMaxSpeed,
      double yP,
      double yMaxSpeed) {
    this(
        driveSubsystem,
        targetX,
        targetY,
        targetAngle,
        positionTolerance,
        angleTolerance,
        false,
        xP,
        xMaxSpeed,
        yP,
        yMaxSpeed);
  }

  /**
   * Precise-stop constructor with per-command translation PID/speed tuning.
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double xP,
      double xMaxSpeed,
      double yP,
      double yMaxSpeed) {
    this(
        driveSubsystem,
        targetX,
        targetY,
        targetAngle,
        0.1,
        2.0,
        false,
        xP,
        xMaxSpeed,
        yP,
        yMaxSpeed);
  }

  /**
   * Precise-stop constructor with custom tolerances.
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      double positionTolerance,
      double angleTolerance) {
    this(driveSubsystem, targetX, targetY, targetAngle, positionTolerance, angleTolerance, false);
  }


  /**
   * Constructor with default tolerances.
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle,
      boolean isContinuous) {
    this(driveSubsystem, targetX, targetY, targetAngle, 0.1, 2.0, isContinuous);
  }

  /**
   * Normal precise-stop constructor.
   */
  public DriveToPoint(
      DriveSubsystem driveSubsystem,
      double targetX,
      double targetY,
      double targetAngle) {
    this(driveSubsystem, targetX, targetY, targetAngle, 0.1, 2.0, false);
  }

  // ===========================================================================================
  // Command Lifecycle
  // ===========================================================================================

  @Override
  public void initialize() {
    xPID.reset();
    yPID.reset();
    turnPID.reset();

    SmartDashboard.putNumber("TARGET_X", targetPose.getX());
    SmartDashboard.putNumber("TARGET_Y", targetPose.getY());
    SmartDashboard.putNumber("TARGET_ANGLE", targetPose.getAngle());
    SmartDashboard.putBoolean("TARGET_IS_CONTINUOUS", isContinuous);

    System.out.println(
        "DriveToPoint: Starting - Target: ("
            + targetPose.getX()
            + ", "
            + targetPose.getY()
            + ", "
            + targetPose.getAngle()
            + "°)"
            + " continuous="
            + isContinuous);
  }

  @Override
  public void execute() {
    Pose currentPose = driveSubsystem.getPose();

    // Translation control
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();

    xPID.setDesiredValue(0);
    yPID.setDesiredValue(0);

    double xSpeed = xPID.calculate(-xError);
    double ySpeed = yPID.calculate(-yError);

    // Rotation control
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());
    double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);

    turnPID.setDesiredValue(0);
    double rotationSpeed = turnPID.calculate(-angleError);

    driveSubsystem.drive(xSpeed, ySpeed, rotationSpeed, true);
  }

  @Override
  public void end(boolean interrupted) {
    // Only stop drivetrain for true stop points
    if (!isContinuous) {
      driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }
  }

  @Override
  public boolean isFinished() {
    Pose currentPose = driveSubsystem.getPose();

    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    double distanceToTarget = Math.sqrt(xError * xError + yError * yError);
    boolean positionOnTarget = distanceToTarget <= positionTolerance;

    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());
    double angleError =
        Math.abs(Calculations.shortestAngularDistance(targetAngle, currentAngle));
    boolean angleOnTarget = angleError <= angleTolerance;

    // Continuous points only care about getting near the point
    if (isContinuous) {
      return positionOnTarget;
    }

    // Stop points require both position and angle
    return positionOnTarget && angleOnTarget;
  }

  // ===========================================================================================
  // Status Methods
  // ===========================================================================================

  public double getDistanceToTarget() {
    Pose currentPose = driveSubsystem.getPose();
    double xError = targetPose.getX() - currentPose.getX();
    double yError = targetPose.getY() - currentPose.getY();
    return Math.sqrt(xError * xError + yError * yError);
  }

  public double getAngleError() {
    Pose currentPose = driveSubsystem.getPose();
    double currentAngle = Calculations.normalizeAngle360(currentPose.getAngle());
    double targetAngle = Calculations.normalizeAngle360(targetPose.getAngle());
    return Calculations.shortestAngularDistance(targetAngle, currentAngle);
  }

  public Pose getTargetPose() {
    return new Pose(targetPose);
  }

  public boolean isContinuous() {
    return isContinuous;
  }
}
