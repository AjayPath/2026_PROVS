package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.APPID;
import frc.robot.utils.Calculations;

public class TurnToAngle extends Command {
  private static final double kTurnP = 0.02;
  private static final double kTurnI = 0.0;
  private static final double kTurnD = 0.0;
  private static final double kMaxRot = 0.75;

  private final DriveSubsystem driveSubsystem;
  private final APPID turnPID;
  private final DoubleSupplier requestedAngleSupplier;
  private final boolean isRelative;

  private double targetAngleDeg;

  public TurnToAngle(
      DriveSubsystem driveSubsystem,
      DoubleSupplier requestedAngleSupplier,
      double angleToleranceDeg,
      boolean isRelative) {
    this.driveSubsystem = driveSubsystem;
    this.requestedAngleSupplier = requestedAngleSupplier;
    this.isRelative = isRelative;

    turnPID = new APPID(kTurnP, kTurnI, kTurnD, angleToleranceDeg);
    turnPID.setMaxOutput(kMaxRot);

    addRequirements(driveSubsystem);
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg) {
    this(driveSubsystem, () -> targetAngleDeg, 2.0, false);
  }

  public TurnToAngle(DriveSubsystem driveSubsystem, double targetAngleDeg, double angleToleranceDeg) {
    this(driveSubsystem, () -> targetAngleDeg, angleToleranceDeg, false);
  }

  public TurnToAngle(
      DriveSubsystem driveSubsystem,
      DoubleSupplier targetAngleSupplier,
      double angleToleranceDeg) {
    this(driveSubsystem, targetAngleSupplier, angleToleranceDeg, false);
  }

  public static TurnToAngle relative(DriveSubsystem driveSubsystem, double deltaAngleDeg) {
    return new TurnToAngle(driveSubsystem, () -> deltaAngleDeg, 2.0, true);
  }

  public static TurnToAngle relative(
      DriveSubsystem driveSubsystem,
      double deltaAngleDeg,
      double toleranceDeg) {
    return new TurnToAngle(driveSubsystem, () -> deltaAngleDeg, toleranceDeg, true);
  }

  @Override
  public void initialize() {
    turnPID.reset();
  }

  @Override
  public void execute() {
    double currentAngle = Calculations.normalizeAngle360(driveSubsystem.getPose().getAngle());
    double requestedAngleDeg = requestedAngleSupplier.getAsDouble();

    if (isRelative) {
      targetAngleDeg = Calculations.normalizeAngle360(currentAngle + requestedAngleDeg);
    } else {
      targetAngleDeg = Calculations.normalizeAngle360(requestedAngleDeg);
    }

    double angleError = Calculations.shortestAngularDistance(targetAngleDeg, currentAngle);

    turnPID.setDesiredValue(0.0);
    double rotCmd = turnPID.calculate(-angleError);

    driveSubsystem.drive(0.0, 0.0, rotCmd, true);
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
