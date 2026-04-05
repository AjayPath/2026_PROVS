// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;

public class DataLog extends SubsystemBase {
  /** Creates a new DataLog. */
  public DataLog() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("AprilTag", Variables.limelight.hasTarget);
    SmartDashboard.putNumber("TID", Variables.limelight.tid);

    SmartDashboard.putNumber("X", Variables.drive.currentX);
    SmartDashboard.putNumber("Y", Variables.drive.currentY);
    SmartDashboard.putNumber("Heading", Variables.drive.heading);
  }
}
