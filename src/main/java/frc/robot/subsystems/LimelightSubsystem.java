package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables;

/**
 * LimelightSubsystem
 *
 * Reads data from Limelight and writes it directly to Variables.limelight
 */
public class LimelightSubsystem extends SubsystemBase {

    private static final String LIMELIGHT_NAME = "limelight-naci";

    private final NetworkTable table;

    public LimelightSubsystem() {
        table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
    }

    @Override
    public void periodic() {

        // Read basic targeting values
        Variables.limelight.tx = table.getEntry("tx").getDouble(0.0);
        Variables.limelight.ty = table.getEntry("ty").getDouble(0.0);
        Variables.limelight.tid = (int) table.getEntry("tid").getDouble(0.0);

        // Target validity
        double tv = table.getEntry("tv").getDouble(0.0);

        // AprilTag pose data (WPILib blue coordinate system)
        double[] botpose = table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);

        // Validate pose data
        if (tv == 1.0 && botpose.length >= 8 && botpose[7] > 0) {

            Variables.limelight.hasTarget = true;

            // Position from Limelight
            Variables.limelight.ll_x = botpose[0];
            Variables.limelight.ll_y = botpose[1];
            Variables.limelight.ll_rot = botpose[5];

            // Number of tags used in the solve
            Variables.limelight.tagCount = botpose[7];

        } else {

            Variables.limelight.hasTarget = false;
            Variables.limelight.tagCount = 0.0;

            // NOTE:
            // We are intentionally NOT resetting ll_x / ll_y
            // so last known pose is preserved
        }
    }
}