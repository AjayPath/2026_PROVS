package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.DriveToPoint;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetPivotPosition;
import frc.robot.commands.ShootSequence;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FloorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoRoutines {
  private final DriveSubsystem robotDrive;
  private final ShooterSubsystem shooterSubsystem;
  private final FeederSubsystem feederSubsystem;
  private final FloorSubsystem floorSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final PivotSubsystem pivotSubsystem;

  public AutoRoutines(
      DriveSubsystem robotDrive,
      ShooterSubsystem shooterSubsystem,
      FeederSubsystem feederSubsystem,
      FloorSubsystem floorSubsystem,
      IntakeSubsystem intakeSubsystem,
      PivotSubsystem pivotSubsystem) {
    this.robotDrive = robotDrive;
    this.shooterSubsystem = shooterSubsystem;
    this.feederSubsystem = feederSubsystem;
    this.floorSubsystem = floorSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.pivotSubsystem = pivotSubsystem;
  }

  public Command getCommandForMode(AutoMode mode) {
    return switch (mode) {
      case RED_RIGHT -> buildRedRightAuto();
      case RED_LEFT -> buildRedLeftAuto();
      case BLUE_RIGHT -> buildBlueRightAuto();
      case BLUE_LEFT -> buildBlueLeftAuto();
    };
  }

  private Command buildRedRightAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.4, -7.3, 90, 0.35, 0.02, true, 1, 1.15, 1, 1.15),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new SetPivotPosition(pivotSubsystem, 108))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -9.4, -5, 90, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.4, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -13.4, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -13.5, -7.3, 90, 0.1, 0.02, true, 0.8, 0.45, 0.8, 0.45),
            new SetPivotPosition(pivotSubsystem, 0)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.7, -7.3, 90, 0.25, 0.02, true, 1, 1.35, 1, 1.35),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new SetPivotPosition(pivotSubsystem, 0))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.7, -5, 90, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.7, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -13.5, -5.75, 135, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5));
  }

  private Command buildRedLeftAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.4, -0.6, 270, 0.35, 0.02, true, 1, 1.15, 1, 1.15),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new SetPivotPosition(pivotSubsystem, 108))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -9.4, -4, 270, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.4, -2.5, 225, 0.5, 0.02, true, 0.8, 0.8, 0.8, 0.8),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -13.5, -2.5, 225, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -13.5, -0.6, 270, 0.1, 0.02, true, 0.8, 0.45, 0.8, 0.45),
            new SetPivotPosition(pivotSubsystem, 0)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.7, -0.6, 270, 0.25, 0.02, true, 1, 1.35, 1, 1.35),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new SetPivotPosition(pivotSubsystem, 0))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.7, -4, 270, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -10.4, -2.5, 225, 0.5, 0.02, true, 0.8, 0.8, 0.8, 0.8),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -13.5, -2.5, 225, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5));
  }

  private Command buildBlueRightAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6.2, -0.6, 270, 0.35, 0.02, true, 1, 0.5, 1, 0.5),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new SetPivotPosition(pivotSubsystem, 108))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -7.2, -4, 270, 0.1, 0.02, true, 0.8, 0.65, 0.8, 0.65),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6, -2.5, 225, 0.5, 0.02, true, 0.8, 0.5, 0.8, 0.5),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -3.5, -2.5, 225, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -3.5, -0.6, 270, 0.1, 0.02, true, 0.8, 0.45, 0.8, 0.45),
            new SetPivotPosition(pivotSubsystem, 0)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6.2, -0.6, 270, 0.25, 0.02, true, 1, 0.5, 1, 0.5),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new SetPivotPosition(pivotSubsystem, 0))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6.2, -4, 270, 0.1, 0.02, true, 0.8, 0.5, 0.8, 0.5),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6, -2.5, 225, 0.5, 0.02, true, 0.8, 0.5, 0.8, 0.5),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -3.5, -2.5, 225, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5));
  }

  private Command buildBlueLeftAuto() {
    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6, -7.45, 90, 0.35, 0.02, true, 1, 0.5, 1, 0.5),
            new SequentialCommandGroup(
                new WaitCommand(0.5),
                new SetPivotPosition(pivotSubsystem, 108))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -7, -4.3, 90, 0.1, 0.02, true, 0.8, 0.45, 0.8, 0.45),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -5, -5.35, 35, 0.1, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -2.5, -5.35, 35, 0.1, 0.02, true, 0.8, 0.5, 0.8, 0.5),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -2.5, -7.45, 90, 0.1, 0.02, true, 0.8, 0.45, 0.8, 0.45),
            new SetPivotPosition(pivotSubsystem, 0)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6.5, -7.3, 90, 0.1, 0.02, true, 1, 1.35, 1, 1.35),
            new SequentialCommandGroup(
                new WaitCommand(1),
                new SetPivotPosition(pivotSubsystem, 0))),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6.5, -4, 90, 0.1, 0.02, true, 0.8, 0.85, 0.8, 0.85),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -6.5, -5.35, 35, 0.25, 0.02, true, 0.8, 0.4, 0.8, 0.4),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ParallelDeadlineGroup(
            new DriveToPoint(robotDrive, -2.5, -5.35, 35, 0.25, 0.02, true, 0.8, 0.5, 0.8, 0.5),
            new RunIntake(intakeSubsystem, pivotSubsystem, 93, 108)),

        new ShootSequence(
            shooterSubsystem,
            feederSubsystem,
            floorSubsystem,
            robotDrive,
            intakeSubsystem,
            pivotSubsystem).withTimeout(3.5));
  }
}
