package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Variables;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.utils.APTree;

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
        new TurnToAngle(
            drive,
            () -> Variables.drive.targetHubAngleDeg,
            2.0,
            forwardSupplier,
            strafeSupplier,
            kShootMoveTranslationScale),
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
}
