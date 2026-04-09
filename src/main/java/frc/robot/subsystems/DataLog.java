package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;
import frc.robot.utils.LimelightToAPOTranslator;
import frc.robot.utils.Pose;

public class DataLog extends SubsystemBase {
  public DataLog() {}

  @Override
public void periodic() {
    SmartDashboard.putBoolean("LL/HasTarget", Variables.limelight.hasTarget);
    SmartDashboard.putNumber("LL/TID", Variables.limelight.tid);
    SmartDashboard.putNumber("LL/TagCount", Variables.limelight.tagCount);
    SmartDashboard.putNumber("LL/LatencyMs", Variables.limelight.latencyMs);

    SmartDashboard.putNumber("LL/X", Variables.limelight.ll_x);
    SmartDashboard.putNumber("LL/Y", Variables.limelight.ll_y);
    SmartDashboard.putNumber("LL/Rot", Variables.limelight.ll_rot);

    SmartDashboard.putNumber("Drive/X", Variables.drive.currentX);
    SmartDashboard.putNumber("Drive/Y", Variables.drive.currentY);
    SmartDashboard.putNumber("Drive/Heading", Variables.drive.heading);
    SmartDashboard.putNumber("Drive/TurnRate", Variables.drive.turnRate);

    Pose translatedPose = LimelightToAPOTranslator.getTranslatedPose();

    SmartDashboard.putNumber("LL_APO/X", translatedPose.getX());
    SmartDashboard.putNumber("LL_APO/Y", translatedPose.getY());
    SmartDashboard.putNumber("LL_APO/Rot", translatedPose.getAngle());

    SmartDashboard.putNumber("Error/X", Variables.limelight.ll_x - Variables.drive.currentX);
    SmartDashboard.putNumber("Error/Y", Variables.limelight.ll_y - Variables.drive.currentY);
    SmartDashboard.putNumber("Error/Rot", Variables.limelight.ll_rot - Variables.drive.heading);

    SmartDashboard.putNumber("Error_APO/X", translatedPose.getX() - Variables.drive.currentX);
    SmartDashboard.putNumber("Error_APO/Y", translatedPose.getY() - Variables.drive.currentY);
    SmartDashboard.putNumber("Error_APO/Rot", translatedPose.getAngle() - Variables.drive.heading);

    SmartDashboard.putNumber("TurnTarget/Red/X", Constants.TurnTargetConstants.kRedHubX);
    SmartDashboard.putNumber("TurnTarget/Red/Y", Constants.TurnTargetConstants.kRedHubY);
    SmartDashboard.putNumber("TurnTarget/Blue/X", Constants.TurnTargetConstants.kBlueHubX);
    SmartDashboard.putNumber("TurnTarget/Blue/Y", Constants.TurnTargetConstants.kBlueHubY);

    SmartDashboard.putString("TurnTarget/SelectedName", RobotContainer.getSelectedTurnTargetName());
    SmartDashboard.putNumber("TurnTarget/SelectedX", RobotContainer.getSelectedTurnTargetX());
    SmartDashboard.putNumber("TurnTarget/SelectedY", RobotContainer.getSelectedTurnTargetY());
    SmartDashboard.putNumber("TurnTarget/TargetAngleDeg", Variables.drive.targetHubAngleDeg);
    SmartDashboard.putNumber("TurnTarget/AngleErrorDeg", Variables.drive.targetHubAngleErrorDeg);
  }
}
