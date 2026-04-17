// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Pose;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String LIMELIGHT_NAME = "limelight-naci";

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    initializeGyro();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
public void robotPeriodic() {
  CommandScheduler.getInstance().run();
  m_robotContainer.periodic();
  double currentHeadingDeg = m_robotContainer.m_robotDrive.getHeading();
  double currentYawRateDegPerSec = m_robotContainer.m_robotDrive.getTurnRate();

  // Give MegaTag2 the robot heading from the gyro/odometry
  LimelightHelpers.SetRobotOrientation(
      LIMELIGHT_NAME,
      currentHeadingDeg,
      currentYawRateDegPerSec, 0.0, 0.0, 0.0, 0.0);

  var llMeasurement =
      LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_NAME);

  if (llMeasurement != null) {
    Variables.limelight.hasTarget = llMeasurement.tagCount > 0;
    Variables.limelight.tagCount = llMeasurement.tagCount;
    Variables.limelight.latencyMs = llMeasurement.latency;
    Variables.limelight.ll_x = llMeasurement.pose.getX();
    Variables.limelight.ll_y = llMeasurement.pose.getY();
    Variables.limelight.ll_rot = llMeasurement.pose.getRotation().getDegrees();
  } else {
    Variables.limelight.hasTarget = false;
    Variables.limelight.tagCount = 0;
    Variables.limelight.latencyMs = 0;
  }

  // Conservative acceptance filter
  boolean reject = false;

  if (llMeasurement == null || llMeasurement.tagCount == 0) {
    reject = true;
  } else if (llMeasurement.tagCount == 1 && llMeasurement.rawFiducials.length == 1) {
    if (llMeasurement.rawFiducials[0].ambiguity > 0.7) reject = true;
    if (llMeasurement.rawFiducials[0].distToCamera > 3.0) reject = true;
  }

  // Do NOT hard set pose every frame unless you are initializing
  if (!reject) {
    // Apply accepted Limelight translation corrections into odometry.
    // Heading remains gyro-driven.
    m_robotContainer.m_robotDrive.correctPositionFromLimelight();
  }
}

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    applyDashboardStartingPose();
  }

  @Override
  public void disabledPeriodic() {
    applyDashboardStartingPose();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  private void initializeGyro() {
  DriveSubsystem drive = m_robotContainer.getDriveSubsystem();

  new Thread(() -> {
      try {
          Thread.sleep(1000);
      } catch (InterruptedException e) {
          e.printStackTrace();
      }

      // initialize only, do not zero yaw here
      drive.initializeGyro();
      applyDashboardStartingPose();
  }).start();
}

  private void applyDashboardStartingPose() {
    double selectedHeadingDeg = m_robotContainer.getSelectedStartingHeadingDeg();
    double selectedX = m_robotContainer.getSelectedStartingX();
    double selectedY = m_robotContainer.getSelectedStartingY();

    DriveSubsystem drive = m_robotContainer.getDriveSubsystem();
    drive.setGyroYaw(selectedHeadingDeg);
    drive.setPose(new Pose(selectedX, selectedY, selectedHeadingDeg));
  }
  
}
