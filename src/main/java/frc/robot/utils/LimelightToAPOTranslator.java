package frc.robot.utils;

import frc.robot.Variables;

/**
 * Converts Limelight field pose values into APOdometry's field frame.
 *
 * Current measured mapping:
 *   odomX = -llX
 *   odomY = -llY
 *
 * Rotation is left configurable because position mapping is proven,
 * but heading mapping should still be verified on-robot.
 */
public final class LimelightToAPOTranslator {

    // Position mapping determined from your tests
    private static final boolean INVERT_X = true;
    private static final boolean INVERT_Y = true;

    // Leave rotation configurable until you finish the heading test
    private static final boolean INVERT_ROTATION = false;
    private static final double ROTATION_OFFSET_DEG = 0.0;

    private LimelightToAPOTranslator() {}

    /**
     * Convert raw Limelight coordinates into APOdometry coordinates.
     */
    public static Pose toAPOPose(double llX, double llY, double llRotDeg) {
        double x = INVERT_X ? -llX : llX;
        double y = INVERT_Y ? -llY : llY;

        double rot = INVERT_ROTATION ? -llRotDeg : llRotDeg;
        rot += ROTATION_OFFSET_DEG;
        rot = Calculations.normalizeAngle360(rot);

        return new Pose(x, y, rot);
    }

    /**
     * Convert directly from your shared Variables.limelight values.
     */
    public static Pose getTranslatedPose() {
        return toAPOPose(
            Variables.limelight.ll_x,
            Variables.limelight.ll_y,
            Variables.limelight.ll_rot
        );
    }

    /**
     * Convenience helper if you only want translated X.
     */
    public static double getTranslatedX() {
        return INVERT_X ? -Variables.limelight.ll_x : Variables.limelight.ll_x;
    }

    /**
     * Convenience helper if you only want translated Y.
     */
    public static double getTranslatedY() {
        return INVERT_Y ? -Variables.limelight.ll_y : Variables.limelight.ll_y;
    }

    /**
     * Convenience helper if you only want translated heading.
     */
    public static double getTranslatedRotationDeg() {
        double rot = INVERT_ROTATION ? -Variables.limelight.ll_rot : Variables.limelight.ll_rot;
        rot += ROTATION_OFFSET_DEG;
        return Calculations.normalizeAngle360(rot);
    }
}