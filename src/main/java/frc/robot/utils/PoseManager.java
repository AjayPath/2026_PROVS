package frc.robot.utils;

import frc.robot.Variables;
import frc.robot.subsystems.DriveSubsystem;

public class PoseManager {

    private final DriveSubsystem drive;

    // tuning parameters
    private static final double MIN_UPDATE_DISTANCE = 0.1; // meters

    public PoseManager(DriveSubsystem drive) {
        this.drive = drive;
    }

    /**
     * Call every loop
     * Applies Limelight pose if valid
     */
    public void update() {

        if (!Variables.limelight.hasTarget) return;

        Pose current = drive.getPose();

        double dx = Variables.limelight.ll_x - current.getX();
        double dy = Variables.limelight.ll_y - current.getY();

        double dist = Math.hypot(dx, dy);

        // Only update if meaningful difference
        if (dist > MIN_UPDATE_DISTANCE) {

            drive.setPose(new Pose(
                Variables.limelight.ll_x,
                Variables.limelight.ll_y,
                Variables.drive.heading // keep gyro rotation
            ));
        }
    }

    // Blend
    // public void update() {

    //     if (!Variables.limelight.hasTarget) return;

    //     Pose current = drive.getPose();

    //     double dx = Variables.limelight.ll_x - current.getX();
    //     double dy = Variables.limelight.ll_y - current.getY();

    //     double dist = Math.hypot(dx, dy);

    //     // Reject clearly bad vision
    //     if (dist > 2.0) return;

    //     double alpha = 0.05;

    //     double newX = current.getX() + dx * alpha;
    //     double newY = current.getY() + dy * alpha;

    //     drive.setPose(new Pose(
    //         newX,
    //         newY,
    //         Variables.drive.heading
    //     ));
    // }
}