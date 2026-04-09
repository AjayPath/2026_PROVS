// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystems.DataLog;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Calculations;
import frc.robot.utils.LimelightHelpers;
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
  private static final String RED_ALLIANCE = "Red";
  private static final String BLUE_ALLIANCE = "Blue";
  private static final String RED_HUB = "Red Hub";
  private static final String BLUE_HUB = "Blue Hub";

  /// The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //public final LimelightSubsystem m_limelight = new LimelightSubsystem();
  public final DataLog m_datalog = new DataLog();
  private final SendableChooser<String> m_allianceChooser = new SendableChooser<>();
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
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  private void configureDashboard() {
    m_allianceChooser.setDefaultOption(RED_ALLIANCE, RED_ALLIANCE);
    m_allianceChooser.addOption(BLUE_ALLIANCE, BLUE_ALLIANCE);
    SmartDashboard.putData("Alliance Start", m_allianceChooser);

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    new Trigger(m_driverController::getLeftBumperButton)
      .whileTrue(
        new RunCommand(
          () -> m_robotDrive.drive(LimelightHelpers.getTY("limelight-naci") * -0.1, -m_driverController.getLeftX(), LimelightHelpers.getTX("limelight-naci") * -0.05, false), m_robotDrive)
      );

    new Trigger(m_driverController::getBButton)
      .whileTrue(new TurnToAngle(m_robotDrive, () -> Variables.drive.targetHubAngleDeg, 2.0));

    new Trigger(m_driverController::getAButton)
      .whileTrue(new SequentialCommandGroup(
        new DriveToPoint(m_robotDrive, -14.75, -4, 0, 0.1, 2.0, true),
        new DriveToPoint(m_robotDrive, -14.75, -1.2, 0, 0.1, 2.0, true)
      ).repeatedly());


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
    String alliance = m_allianceChooser.getSelected();
    return BLUE_ALLIANCE.equals(alliance) ? 180.0 : 0.0;
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

    Variables.drive.targetHubAngleDeg =
        Calculations.normalizeAngle360(Math.toDegrees(Math.atan2(dy, dx)));
    Variables.drive.targetHubAngleErrorDeg =
        Calculations.shortestAngularDistance(
            Variables.drive.targetHubAngleDeg,
            Calculations.normalizeAngle360(currentPose.getAngle()));
    //m_poseManager.update();
  }
}
