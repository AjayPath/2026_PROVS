// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Variables;

// /**
//  * LimelightSubsystem
//  *
//  * Reads data from Limelight and writes it directly to Variables.limelight
//  */
// public class LimelightSubsystem extends SubsystemBase {

//     private static final String LIMELIGHT_NAME = "limelight-naci";

//     private final NetworkTable table;

//     public LimelightSubsystem() {
//         table = NetworkTableInstance.getDefault().getTable(LIMELIGHT_NAME);
//     }

//     @Override
//     public void periodic() {

//         // Send robot orientation back to Limelight every loop
//         // Format: yaw, yawRate, pitch, pitchRate, roll, rollRate
//         table.getEntry("robot_orientation_set").setDoubleArray(new double[] {
//     Variables.drive.visionYaw,
//     Variables.drive.turnRate,
//     0.0,
//     0.0,
//     0.0,
//     0.0
// });

//         Variables.limelight.tx = table.getEntry("tx").getDouble(0.0);
//         Variables.limelight.ty = table.getEntry("ty").getDouble(0.0);
//         Variables.limelight.tid = table.getEntry("tid").getDouble(0.0);
//         Variables.limelight.pipelineIndex =
//             (int) table.getEntry("getpipe").getDouble(-1);

//         double tv = table.getEntry("tv").getDouble(0.0);

//         double[] botpose =
//             table.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[0]);

//         boolean validPose =
//             tv == 1.0 &&
//             botpose.length >= 8 &&
//             botpose[7] > 0;

//         if (validPose) {
//             Variables.limelight.hasTarget = true;

//             Variables.limelight.ll_x = botpose[0];
//             Variables.limelight.ll_y = botpose[1];
//             Variables.limelight.ll_rot = botpose[5];
//             Variables.limelight.latencyMs = botpose[6];
//             Variables.limelight.tagCount = botpose[7];

//         } else {
//             Variables.limelight.hasTarget = false;
//             Variables.limelight.tagCount = 0.0;
//         }
//     }
// }