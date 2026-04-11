package frc.robot.commands;

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

public class ShootSequence extends SequentialCommandGroup {
  private static final APTree SHOOTER_RPS_BY_DISTANCE = buildShooterRpsTable();

  public ShootSequence(
      ShooterSubsystem shooter,
      FeederSubsystem feeder,
      FloorSubsystem floor,
      DriveSubsystem drive,
      IntakeSubsystem intake,
      PivotSubsystem pivot
  ) {
    addCommands(

        new ParallelCommandGroup(

        new TurnToAngle(drive, () -> Variables.drive.targetHubAngleDeg, 2.0),

        new ParallelCommandGroup(
          // Compute shot speed from straight-line distance to selected hub.
          new RunCommand(
              () -> Variables.shooterRPS = SHOOTER_RPS_BY_DISTANCE.GetValue(Variables.distanceMeters)),

          // Shooter runs the entire time, never interrupted
          new SetShooterRPS(shooter),

          new SequentialCommandGroup(
            // Wait until shooter is up to speed before feeding
            new WaitUntilCommand(() -> shooter.atTargetSpeed()),

            // Then run floor and feeder
            new ParallelCommandGroup(
              new SetFloorRPS(floor, 40),
              new SetFeederRPS(feeder, 90),
                new RunIntake(intake, pivot,30, 70)
            )
          )
        )
        )
      );
  }

  private static APTree buildShooterRpsTable() {
    APTree table = new APTree();
    table.InsertValues(new double[][] {
        // distance meters, shooter RPS
        {1.5, 50.0},
        {2.0, 53.0},
        {2.5, 56.0},
        {3.0, 59.0},
        {3.5, 62.0}
    });
    return table;
  }
}
