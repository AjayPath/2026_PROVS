// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.AutoSelector;
import frc.robot.commands.PassSequence;
import frc.robot.commands.Purge;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.ShootAndMoveSequence;
import frc.robot.commands.ShootSequence;
import frc.robot.commands.StaticShootSequence;
import frc.robot.subsystems.DataLog;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.Calculations;
import frc.robot.utils.Pose;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  /// The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final FloorSubsystem s_floorSubsystem = new FloorSubsystem();
  private final FeederSubsystem s_feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem s_pivotSubsystem = new PivotSubsystem();

  private final AutoSelector autoSelector = new AutoSelector();
  private final AutoRoutines autoRoutines =
      new AutoRoutines(
          m_robotDrive,
          s_shooterSubsystem,
          s_feederSubsystem,
          s_floorSubsystem,
          s_intakeSubsystem,
          s_pivotSubsystem);

  public final DataLog m_datalog =
      new DataLog(
          autoSelector::getSelectedHubName,
          autoSelector::getSelectedHubX,
          autoSelector::getSelectedHubY);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDashboard();
    configureBindings();

    // Configure default commands.
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -MathUtil.applyDeadband(
                        getAllianceAdjustedDriverAxis(m_driverController.getLeftY()),
                        OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        getAllianceAdjustedDriverAxis(m_driverController.getLeftX()),
                        OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(),
                        OIConstants.kDriveDeadband),
                    true),
            m_robotDrive));
  }

  private void configureDashboard() {
    autoSelector.publishToDashboard();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CoSmmandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2)
        .whileTrue(
            new ShootSequence(
                s_shooterSubsystem,
                s_feederSubsystem,
                s_floorSubsystem,
                m_robotDrive,
                s_intakeSubsystem,
                s_pivotSubsystem
        ));

    new Trigger(m_driverController::getRightBumperButton)
        .whileTrue(
            new PassSequence(
                s_shooterSubsystem,
                s_feederSubsystem,
                s_floorSubsystem,
                m_robotDrive,
                s_intakeSubsystem,
                s_pivotSubsystem,
                90,
                () -> autoSelector.isBlueSelected() ? 180.0 : 0.0));

    new Trigger(m_driverController::getYButton)
        .whileTrue(new Purge(s_feederSubsystem, s_shooterSubsystem, s_intakeSubsystem, s_floorSubsystem));

    new Trigger(m_driverController::getXButton)
        .onTrue(new InstantCommand(m_robotDrive::resetGyro, m_robotDrive));

    new Trigger(m_driverController::getAButton)
        .whileTrue(
            new StaticShootSequence(
                s_shooterSubsystem,
                s_feederSubsystem,
                s_floorSubsystem,
                m_robotDrive,
                s_intakeSubsystem,
                s_pivotSubsystem));

    new Trigger(m_driverController::getStartButton)
      .whileTrue(
          new RunCommand(
              () -> m_robotDrive.setGyroYaw(autoSelector.isBlueSelected() ? 180.0 : 0.0)));

    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
        .whileTrue(new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 85, 140));

    new Trigger(m_driverController::getLeftBumperButton)
        .whileTrue(new SetPivotPosition(s_pivotSubsystem, 10));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoRoutines.getCommandForMode(autoSelector.getSelectedMode());
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  public double getSelectedStartingHeadingDeg() {
    return autoSelector.getSelectedStartingHeadingDeg();
  }

  public double getSelectedStartingX() {
    return autoSelector.getSelectedStartingX();
  }

  public double getSelectedStartingY() {
    return autoSelector.getSelectedStartingY();
  }

  private double getAllianceAdjustedDriverAxis(double axisValue) {
    return autoSelector.isBlueSelected() ? -axisValue : axisValue;
  }

  public void periodic() {
    Pose currentPose = m_robotDrive.getPose();
    double dx = autoSelector.getSelectedHubX() - currentPose.getX();
    double dy = autoSelector.getSelectedHubY() - currentPose.getY();
    Variables.distanceMeters = Math.hypot(dx, dy);

    Variables.drive.targetHubAngleDeg =
        Calculations.normalizeAngle360(Math.toDegrees(Math.atan2(dy, dx)));
    Variables.drive.targetHubAngleErrorDeg =
        Calculations.shortestAngularDistance(
            Variables.drive.targetHubAngleDeg,
            Calculations.normalizeAngle360(currentPose.getAngle()));
  }
}
