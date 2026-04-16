// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPoint;
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
//import frc.robot.subsystems.LimelightSubsystem;
//import frc.robot.utils.PoseManager;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final String RED_HUB = "Red Hub";
  private static final String BLUE_HUB = "Blue Hub";
  private static final String START_HEADING_RED_LEFT = "Red Left (270)";
  private static final String START_HEADING_RED_RIGHT = "Red Right (90)";
  private static final String START_HEADING_BLUE_LEFT = "Blue Left (90)";
  private static final String START_HEADING_BLUE_RIGHT = "Blue Right (270)";

  /// The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem s_shooterSubsystem = new ShooterSubsystem();
  private final FloorSubsystem s_floorSubsystem = new FloorSubsystem();
  private final FeederSubsystem s_feederSubsystem = new FeederSubsystem();
  private final IntakeSubsystem s_intakeSubsystem = new IntakeSubsystem();
  private final PivotSubsystem s_pivotSubsystem = new PivotSubsystem();
  //public final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public final DataLog m_datalog = new DataLog();
  private final SendableChooser<String> m_startHeadingChooser = new SendableChooser<>();
  private static final SendableChooser<String> m_turnTargetChooser = new SendableChooser<>();

  //private final PoseManager m_poseManager = new PoseManager(m_robotDrive);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  XboxController m_driverController =
      new XboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureDashboard();

    // Configure the trigger bindings
    configureBindings();

        // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
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
    m_startHeadingChooser.setDefaultOption(START_HEADING_RED_LEFT, START_HEADING_RED_LEFT);
    m_startHeadingChooser.addOption(START_HEADING_RED_RIGHT, START_HEADING_RED_RIGHT);
    m_startHeadingChooser.addOption(START_HEADING_BLUE_LEFT, START_HEADING_BLUE_LEFT);
    m_startHeadingChooser.addOption(START_HEADING_BLUE_RIGHT, START_HEADING_BLUE_RIGHT);
    SmartDashboard.putData("Start Heading", m_startHeadingChooser);

    m_turnTargetChooser.setDefaultOption(RED_HUB, RED_HUB);
    m_turnTargetChooser.addOption(BLUE_HUB, BLUE_HUB);
    SmartDashboard.putData("Turn Target", m_turnTargetChooser);
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
      .whileTrue(new ShootAndMoveSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem,
          m_robotDrive,
          s_intakeSubsystem,
          s_pivotSubsystem,
          () -> getAllianceAdjustedDriverAxis(m_driverController.getLeftY()),
          () -> getAllianceAdjustedDriverAxis(m_driverController.getLeftX())));

      new Trigger(m_driverController::getRightBumperButton)
      .whileTrue(new PassSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem, 90,
          () -> isBlueStartHeadingSelected() ? 0.0 : 180.0));

     new Trigger(m_driverController::getYButton)
      .whileTrue(new Purge(s_feederSubsystem, s_shooterSubsystem, s_intakeSubsystem, s_floorSubsystem));

     new Trigger(m_driverController::getAButton)
      .whileTrue(new StaticShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem));

    new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2)
      .whileTrue(new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 85, 114));

      new Trigger(m_driverController::getLeftBumperButton)
      .whileTrue(new SetPivotPosition(s_pivotSubsystem, 10));

    new Trigger(m_driverController::getXButton)
      .whileTrue(
        new SequentialCommandGroup(
          new ParallelDeadlineGroup(
            new DriveToPoint(m_robotDrive, -10.4, -7.3, 90, 0.35, 0.02, true, 1, 1.15, 1, 1.15), 
            new SequentialCommandGroup(
              new WaitCommand(0.5),
              new SetPivotPosition(s_pivotSubsystem, 108))
            ),
          
          new ParallelDeadlineGroup(
            new DriveToPoint(m_robotDrive, -9.4, -5, 90, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65),
            new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 93, 108)),
          new DriveToPoint(m_robotDrive, -10.4, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
          new DriveToPoint(m_robotDrive, -13.4, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
          new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem).withTimeout(3.5),
          new ParallelDeadlineGroup(
            new DriveToPoint(m_robotDrive, -13.5, -7.3, 90, 0.1, 0.02, true, 0.8, 0.45, 0.8, 0.45),
              new SetPivotPosition(s_pivotSubsystem, 0)),
        new ParallelDeadlineGroup(
          new DriveToPoint(m_robotDrive, -10.7, -7.3, 90, 0.25, 0.02, true, 1, 1.35, 1, 1.35), 
          new SequentialCommandGroup(
              new WaitCommand(1),
              new SetPivotPosition(s_pivotSubsystem, 0))
            ),
        new ParallelDeadlineGroup(
          new DriveToPoint(m_robotDrive, -10.7, -5, 90, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85), 
           new RunIntake(s_intakeSubsystem, s_pivotSubsystem, 93, 108)),
        new DriveToPoint(m_robotDrive, -10.7, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
        new DriveToPoint(m_robotDrive, -13.5, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
        new ShootSequence(s_shooterSubsystem, s_feederSubsystem, s_floorSubsystem, m_robotDrive, s_intakeSubsystem, s_pivotSubsystem).withTimeout(3.5)
      ));

  }

/**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_robotDrive;
  }

  public double getSelectedStartingHeadingDeg() {
    String selectedStartHeading = m_startHeadingChooser.getSelected();

    if (START_HEADING_RED_LEFT.equals(selectedStartHeading)) {
      return 270.0;
    }
    if (START_HEADING_RED_RIGHT.equals(selectedStartHeading)) {
      return 90.0;
    }
    if (START_HEADING_BLUE_LEFT.equals(selectedStartHeading)) {
      return 90.0;
    }
    if (START_HEADING_BLUE_RIGHT.equals(selectedStartHeading)) {
      return 270.0;
    }

    return 270.0;
  }

  private double getAllianceAdjustedDriverAxis(double axisValue) {
    return isBlueStartHeadingSelected() ? -axisValue : axisValue;
  }

  private boolean isBlueStartHeadingSelected() {
    String selectedStartHeading = m_startHeadingChooser.getSelected();
    return START_HEADING_BLUE_LEFT.equals(selectedStartHeading)
        || START_HEADING_BLUE_RIGHT.equals(selectedStartHeading);
  }

  public static String getSelectedTurnTargetName() {
    String selectedTarget = m_turnTargetChooser.getSelected();
    return BLUE_HUB.equals(selectedTarget) ? BLUE_HUB : RED_HUB;
  }

  public static double getSelectedTurnTargetX() {
    return BLUE_HUB.equals(getSelectedTurnTargetName())
        ? Constants.TurnTargetConstants.kBlueHubX
        : Constants.TurnTargetConstants.kRedHubX;
  }

  public static double getSelectedTurnTargetY() {
    return BLUE_HUB.equals(getSelectedTurnTargetName())
        ? Constants.TurnTargetConstants.kBlueHubY
        : Constants.TurnTargetConstants.kRedHubY;
  }

  public void periodic() {
    Pose currentPose = m_robotDrive.getPose();
    double dx = getSelectedTurnTargetX() - currentPose.getX();
    double dy = getSelectedTurnTargetY() - currentPose.getY();
    Variables.distanceMeters = Math.hypot(dx, dy);

    Variables.drive.targetHubAngleDeg =
        Calculations.normalizeAngle360(Math.toDegrees(Math.atan2(dy, dx)));
    Variables.drive.targetHubAngleErrorDeg =
        Calculations.shortestAngularDistance(
            Variables.drive.targetHubAngleDeg,
            Calculations.normalizeAngle360(currentPose.getAngle()));
  }
}
