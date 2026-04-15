package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.Variables;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.APTree;
import frc.robot.utils.Calculations;

public class ShootAndMoveSequence extends ParallelCommandGroup {
  private static final APTree SHOOTER_RPS_BY_DISTANCE = buildShooterRpsTable();
  private static final double kShootMoveTranslationScale = 0.2;

  public ShootAndMoveSequence(
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      FloorSubsystem floor,
      DriveSubsystem drive,
      IntakeSubsystem intake,
      PivotSubsystem pivot,
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier) {
    addCommands(
        new FaceHubWhileDriving(drive, forwardSupplier, strafeSupplier, kShootMoveTranslationScale),
        new ParallelCommandGroup(
            new RunCommand(
                () -> Variables.shooterRPS =
                    SHOOTER_RPS_BY_DISTANCE.GetValue(Variables.distanceMeters)),
            new SetShooterRPS(shooter),
            new SequentialCommandGroup(
                new WaitUntilCommand(shooter::atTargetSpeed),
                new ParallelCommandGroup(
                    new SetFloorRPS(floor, 40),
                    new SetFeederRPS(feeder, 90),
                    new RunIntake(intake, pivot, 30, 70)))));
  }

  private static APTree buildShooterRpsTable() {
    APTree table = new APTree();
    table.InsertValues(new double[][] {
        {1.35, 52.5},
        {2.0, 60.0},
        {2.5, 65.0},
        {3.0, 70.0},
        {3.7, 85.0},
        {4.2, 93.0}
    });
    return table;
  }

  private static class FaceHubWhileDriving extends Command {
    private static final double kTurnP = 0.02;
    private static final double kTurnI = 0.0;
    private static final double kTurnD = 0.0;
    private static final double kMaxRot = 0.75;

    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier strafeSupplier;
    private final double translationScale;
    private final APPID turnPID;

    FaceHubWhileDriving(
        DriveSubsystem driveSubsystem,
        DoubleSupplier forwardSupplier,
        DoubleSupplier strafeSupplier,
        double translationScale) {
      this.driveSubsystem = driveSubsystem;
      this.forwardSupplier = forwardSupplier;
      this.strafeSupplier = strafeSupplier;
      this.translationScale = MathUtil.clamp(translationScale, 0.0, 1.0);

      turnPID = new APPID(kTurnP, kTurnI, kTurnD, 2.0);
      turnPID.setMaxOutput(kMaxRot);

      addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
      turnPID.reset();
    }

    @Override
    public void execute() {
      double xCmd =
          -MathUtil.applyDeadband(forwardSupplier.getAsDouble(), OIConstants.kDriveDeadband)
              * translationScale;
      double yCmd =
          -MathUtil.applyDeadband(strafeSupplier.getAsDouble(), OIConstants.kDriveDeadband)
              * translationScale;

      double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());
      double targetAngle = Calculations.normalizeAngle360(Variables.drive.targetHubAngleDeg);
      double angleError = Calculations.shortestAngularDistance(targetAngle, currentAngle);

      turnPID.setDesiredValue(0.0);
      double rotCmd = turnPID.calculate(-angleError);

      driveSubsystem.drive(xCmd, yCmd, rotCmd, true);
    }

    @Override
    public void end(boolean interrupted) {
      driveSubsystem.drive(0.0, 0.0, 0.0, true);
    }

    @Override
    public boolean isFinished() {
      return false;
    }
  }
}
