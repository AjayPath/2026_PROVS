// // // TEST 1

// package frc.robot.utils;

// import frc.robot.Variables;
// import frc.robot.subsystems.DriveSubsystem;

// public class PoseManager {

//     private final DriveSubsystem drive;

//     // tuning parameters
//     private static final double MIN_UPDATE_DISTANCE = 0.1; // meters

//     public PoseManager(DriveSubsystem drive) {
//         this.drive = drive;
//     }

//     /**
//      * Call every loop
//      * Applies Limelight pose if valid
//      */
//     public void update() {

//         if (!Variables.limelight.hasTarget) return;

//         Pose current = drive.getPose();

//         double dx = Variables.limelight.ll_x - current.getX();
//         double dy = Variables.limelight.ll_y - current.getY();

//         double dist = Math.hypot(dx, dy);

//         // Only update if meaningful difference
//         if (dist > MIN_UPDATE_DISTANCE) {

//             drive.setPose(new Pose(
//                 Variables.limelight.ll_x,
//                 Variables.limelight.ll_y,
//                 Variables.drive.heading // keep gyro rotation
//             ));
//         }
//     }

//     // Blend
//     // public void update() {

//     //     if (!Variables.limelight.hasTarget) return;

//     //     Pose current = drive.getPose();

//     //     double dx = Variables.limelight.ll_x - current.getX();
//     //     double dy = Variables.limelight.ll_y - current.getY();

//     //     double dist = Math.hypot(dx, dy);

//     //     // Reject clearly bad vision
//     //     if (dist > 2.0) return;

//     //     double alpha = 0.05;

//     //     double newX = current.getX() + dx * alpha;
//     //     double newY = current.getY() + dy * alpha;

//     //     drive.setPose(new Pose(
//     //         newX,
//     //         newY,
//     //         Variables.drive.heading
//     //     ));
//     //}
// }

// // package frc.robot.utils;

// // import edu.wpi.first.wpilibj.Timer;
// // import frc.robot.Variables;
// // import frc.robot.subsystems.DriveSubsystem;

// // public class PoseManager {

// //     private final DriveSubsystem drive;

// //     // ---------------------------------------------------------
// //     // Normal correction tuning
// //     // ---------------------------------------------------------
// //     private static final double MIN_CORRECTION_DISTANCE_M = 0.03;
// //     private static final double MAX_ACCEPTED_VISION_ERROR_M = 1.50;

// //     private static final double MIN_TAG_COUNT = 1.0;
// //     private static final double MAX_AVG_TAG_DISTANCE_M = 4.5;

// //     private static final double PIPELINE_SWITCH_HOLDOFF_SEC = 0.20;

// //     private static final double MAX_RAW_FRAME_JUMP_M = 0.12;
// //     private static final int MIN_STABLE_FRAMES = 3;

// //     private static final double MIN_ALPHA = 0.08;
// //     private static final double MAX_ALPHA = 0.35;

// //     private static final double HIGH_TURN_RATE_DEG_PER_SEC = 120.0;

// //     // ---------------------------------------------------------
// //     // Initial pose set tuning
// //     // ---------------------------------------------------------
// //     private static final int MIN_INIT_STABLE_FRAMES = 5;
// //     private static final double MIN_INIT_TAG_COUNT = 1.0;
// //     private static final double MAX_INIT_AVG_TAG_DISTANCE_M = 3.5;

// //     // ---------------------------------------------------------
// //     // State
// //     // ---------------------------------------------------------
// //     private int lastPipelineIndex = -1;
// //     private double lastPipelineChangeSec = -1.0;

// //     private double lastRawVisionX = 0.0;
// //     private double lastRawVisionY = 0.0;
// //     private boolean hasLastRawVision = false;

// //     private int stableFrameCount = 0;
// //     private boolean poseInitialized = false;

// //     public PoseManager(DriveSubsystem drive) {
// //         this.drive = drive;
// //     }

// //     /**
// //      * Call every loop.
// //      * Behavior:
// //      * 1) Wait for stable vision
// //      * 2) If pose not initialized yet, hard-set initial pose once
// //      * 3) After initialization, softly blend vision into odometry
// //      */
// //     public void update() {
// //         Pose current = drive.getPose();

// //         // ---------------------------------------------------------
// //         // Default debug values every loop
// //         // ---------------------------------------------------------
// //         Variables.debug.odomRawX = current.getX();
// //         Variables.debug.odomRawY = current.getY();
// //         Variables.debug.correctedX = current.getX();
// //         Variables.debug.correctedY = current.getY();
// //         Variables.debug.visionDx = 0.0;
// //         Variables.debug.visionDy = 0.0;
// //         Variables.debug.visionErrorMeters = 0.0;
// //         Variables.debug.visionAlpha = 0.0;
// //         Variables.debug.stableFrameCount = stableFrameCount;
// //         Variables.debug.pipelineHoldoff = false;
// //         Variables.debug.visionAccepted = false;
// //         Variables.debug.visionStatus = poseInitialized ? "NO_ACTION" : "WAITING_FOR_INITIAL_POSE";

// //         // ---------------------------------------------------------
// //         // No target
// //         // ---------------------------------------------------------
// //         if (!Variables.limelight.hasTarget) {
// //             stableFrameCount = 0;
// //             hasLastRawVision = false;
// //             Variables.debug.stableFrameCount = stableFrameCount;
// //             Variables.debug.visionStatus = poseInitialized ? "NO_TARGET" : "WAITING_FOR_INITIAL_POSE";
// //             return;
// //         }

// //         double now = Timer.getFPGATimestamp();

// //         // ---------------------------------------------------------
// //         // Pipeline switch holdoff
// //         // ---------------------------------------------------------
// //         int currentPipeline = Variables.limelight.pipelineIndex;
// //         if (currentPipeline != lastPipelineIndex) {
// //             lastPipelineIndex = currentPipeline;
// //             lastPipelineChangeSec = now;
// //             stableFrameCount = 0;

// //             lastRawVisionX = Variables.limelight.ll_x;
// //             lastRawVisionY = Variables.limelight.ll_y;
// //             hasLastRawVision = true;

// //             Variables.debug.stableFrameCount = stableFrameCount;
// //             Variables.debug.visionStatus = "PIPELINE_SWITCH";
// //             return;
// //         }

// //         if (lastPipelineChangeSec > 0.0 &&
// //             (now - lastPipelineChangeSec) < PIPELINE_SWITCH_HOLDOFF_SEC) {
// //             Variables.debug.pipelineHoldoff = true;
// //             Variables.debug.stableFrameCount = stableFrameCount;
// //             Variables.debug.visionStatus = "PIPELINE_HOLDOFF";
// //             return;
// //         }

// //         // ---------------------------------------------------------
// //         // Basic quality gates
// //         // ---------------------------------------------------------
// //         if (Variables.limelight.tagCount < MIN_TAG_COUNT) {
// //             stableFrameCount = 0;
// //             Variables.debug.stableFrameCount = stableFrameCount;
// //             Variables.debug.visionStatus = "LOW_TAG_COUNT";
// //             return;
// //         }

// //         if (Variables.limelight.avgTagDist > MAX_AVG_TAG_DISTANCE_M) {
// //             stableFrameCount = 0;
// //             Variables.debug.stableFrameCount = stableFrameCount;
// //             Variables.debug.visionStatus = "TAG_TOO_FAR";
// //             return;
// //         }

// //         // ---------------------------------------------------------
// //         // Stable-frame check
// //         // ---------------------------------------------------------
// //         if (hasLastRawVision) {
// //             double rawJump = Math.hypot(
// //                 Variables.limelight.ll_x - lastRawVisionX,
// //                 Variables.limelight.ll_y - lastRawVisionY
// //             );

// //             if (rawJump <= MAX_RAW_FRAME_JUMP_M) {
// //                 stableFrameCount++;
// //             } else {
// //                 stableFrameCount = 0;
// //                 Variables.debug.stableFrameCount = stableFrameCount;
// //                 Variables.debug.visionStatus = "RAW_JUMP_TOO_LARGE";
// //             }
// //         } else {
// //             stableFrameCount = 0;
// //             hasLastRawVision = true;
// //         }

// //         lastRawVisionX = Variables.limelight.ll_x;
// //         lastRawVisionY = Variables.limelight.ll_y;

// //         Variables.debug.stableFrameCount = stableFrameCount;

// //         // ---------------------------------------------------------
// //         // Initial pose set mode
// //         // ---------------------------------------------------------
// //         if (!poseInitialized) {
// //             if (stableFrameCount < MIN_INIT_STABLE_FRAMES) {
// //                 Variables.debug.visionStatus = "WAITING_FOR_INITIAL_POSE";
// //                 return;
// //             }

// //             if (Variables.limelight.tagCount < MIN_INIT_TAG_COUNT) {
// //                 Variables.debug.visionStatus = "INIT_LOW_TAG_COUNT";
// //                 return;
// //             }

// //             if (Variables.limelight.avgTagDist > MAX_INIT_AVG_TAG_DISTANCE_M) {
// //                 Variables.debug.visionStatus = "INIT_TAG_TOO_FAR";
// //                 return;
// //             }

// //             // Hard set initial XY from Limelight, keep gyro heading authoritative
// //             drive.setPose(new Pose(
// //                 Variables.limelight.ll_x,
// //                 Variables.limelight.ll_y,
// //                 Variables.drive.heading
// //             ));

// //             poseInitialized = true;

// //             Variables.debug.correctedX = Variables.limelight.ll_x;
// //             Variables.debug.correctedY = Variables.limelight.ll_y;
// //             Variables.debug.visionAccepted = true;
// //             Variables.debug.visionStatus = "INITIAL_POSE_SET";

// //             return;
// //         }

// //         // ---------------------------------------------------------
// //         // Normal soft correction mode
// //         // ---------------------------------------------------------
// //         if (stableFrameCount < MIN_STABLE_FRAMES) {
// //             Variables.debug.visionStatus = "WAITING_STABLE_FRAMES";
// //             return;
// //         }

// //         double dx = Variables.limelight.ll_x - current.getX();
// //         double dy = Variables.limelight.ll_y - current.getY();
// //         double dist = Math.hypot(dx, dy);

// //         Variables.debug.visionDx = dx;
// //         Variables.debug.visionDy = dy;
// //         Variables.debug.visionErrorMeters = dist;

// //         if (dist < MIN_CORRECTION_DISTANCE_M) {
// //             Variables.debug.visionStatus = "INSIDE_DEADBAND";
// //             return;
// //         }

// //         if (dist > MAX_ACCEPTED_VISION_ERROR_M) {
// //             Variables.debug.visionStatus = "VISION_ERROR_TOO_LARGE";
// //             return;
// //         }

// //         double alpha = computeAlpha();
// //         Variables.debug.visionAlpha = alpha;

// //         double newX = current.getX() + dx * alpha;
// //         double newY = current.getY() + dy * alpha;

// //         drive.setPose(new Pose(
// //             newX,
// //             newY,
// //             Variables.drive.heading
// //         ));

// //         Variables.debug.correctedX = newX;
// //         Variables.debug.correctedY = newY;
// //         Variables.debug.visionAccepted = true;
// //         Variables.debug.visionStatus = "VISION_APPLIED";
// //     }

// //     private double computeAlpha() {
// //         double alpha = 0.12;

// //         // More trust with more tags
// //         if (Variables.limelight.tagCount >= 2.0) {
// //             alpha += 0.08;
// //         }
// //         if (Variables.limelight.tagCount >= 3.0) {
// //             alpha += 0.05;
// //         }

// //         // More trust when tags are close
// //         if (Variables.limelight.avgTagDist < 3.0) {
// //             alpha += 0.05;
// //         }
// //         if (Variables.limelight.avgTagDist < 2.0) {
// //             alpha += 0.05;
// //         }

// //         // More trust when tag span is decent
// //         if (Variables.limelight.tagSpan > 0.8) {
// //             alpha += 0.03;
// //         }

// //         // Less trust when spinning fast
// //         if (Math.abs(Variables.drive.turnRate) > HIGH_TURN_RATE_DEG_PER_SEC) {
// //             alpha *= 0.5;
// //         }

// //         if (alpha < MIN_ALPHA) alpha = MIN_ALPHA;
// //         if (alpha > MAX_ALPHA) alpha = MAX_ALPHA;

// //         return alpha;
// //     }

// //     /**
// //      * Optional manual reset if you want to force re-localization later.
// //      */
// //     public void resetVisionInitialization() {
// //         poseInitialized = false;
// //         stableFrameCount = 0;
// //         hasLastRawVision = false;
// //         lastPipelineIndex = -1;
// //         lastPipelineChangeSec = -1.0;
// //     }

// //     public boolean isPoseInitialized() {
// //         return poseInitialized;
// //     }
// // }

// // package frc.robot.utils;

// // import edu.wpi.first.wpilibj.Timer;
// // import frc.robot.Variables;
// // import frc.robot.subsystems.DriveSubsystem;


// // public class PoseManager {

// //     private final DriveSubsystem drive;

// //     // Only update if the pose difference is meaningful
// //     private static final double MIN_UPDATE_DISTANCE = 0.1; // meters

// //     // Ignore vision briefly after pipeline changes
// //     private static final double PIPELINE_SWITCH_HOLDOFF_SEC = 0.15;

// //     private int lastPipeline = -1;
// //     private double lastPipelineSwitchTime = -1.0;

// //     public PoseManager(DriveSubsystem drive) {
// //         this.drive = drive;
// //     }

// //     /**
// //      * Call every loop
// //      * Applies Limelight pose if valid
// //      */
// //     public void update() {
// //         if (!Variables.limelight.hasTarget) return;

// //         int currentPipeline = Variables.limelight.pipelineIndex;
// //         double now = Timer.getFPGATimestamp();

// //         // If pipeline changed, start short holdoff
// //         if (currentPipeline != lastPipeline) {
// //             lastPipeline = currentPipeline;
// //             lastPipelineSwitchTime = now;
// //             return;
// //         }

// //         // Ignore vision briefly after pipeline switch
// //         if (lastPipelineSwitchTime > 0 &&
// //             (now - lastPipelineSwitchTime) < PIPELINE_SWITCH_HOLDOFF_SEC) {
// //             return;
// //         }

// //         Pose current = drive.getPose();

// //         double dx = Variables.limelight.ll_x - current.getX();
// //         double dy = Variables.limelight.ll_y - current.getY();

// //         double dist = Math.hypot(dx, dy);

// //         // Only update if meaningful difference
// //         if (dist > MIN_UPDATE_DISTANCE) {
// //             drive.setPose(new Pose(
// //                 Variables.limelight.ll_x,
// //                 Variables.limelight.ll_y,
// //                 Variables.drive.heading // keep gyro rotation
// //             ));
// //         }
// //     }
// // }

// // package frc.robot.utils;

// // import frc.robot.Variables;
// // import frc.robot.subsystems.DriveSubsystem;

// // public class PoseManager {

// //     private final DriveSubsystem drive;

// //     // Only update if difference is meaningful
// //     private static final double MIN_UPDATE_DISTANCE = 0.1; // meters

// //     // One-time startup initialization
// //     private boolean poseInitialized = false;

// //     public PoseManager(DriveSubsystem drive) {
// //         this.drive = drive;
// //     }

// //     /**
// //      * Call every loop
// //      */
// //     public void update() {
// //         if (!Variables.limelight.hasTarget) return;

// //         // -------------------------------------------------
// //         // Startup: initialize pose once from Limelight
// //         // -------------------------------------------------
// //         if (!poseInitialized && Variables.limelight.tagCount >= 1) {
// //             drive.setPose(new Pose(
// //                 Variables.limelight.ll_x,
// //                 Variables.limelight.ll_y,
// //                 Variables.drive.heading // keep gyro heading
// //             ));
// //             poseInitialized = true;
// //             return;
// //         }

// //         // -------------------------------------------------
// //         // Normal snap update logic
// //         // -------------------------------------------------
// //         Pose current = drive.getPose();

// //         double dx = Variables.limelight.ll_x - current.getX();
// //         double dy = Variables.limelight.ll_y - current.getY();

// //         double dist = Math.hypot(dx, dy);

// //         if (dist > MIN_UPDATE_DISTANCE) {
// //             drive.setPose(new Pose(
// //                 Variables.limelight.ll_x,
// //                 Variables.limelight.ll_y,
// //                 Variables.drive.heading // keep gyro heading
// //             ));
// //         }
// //     }

// //     public boolean isPoseInitialized() {
// //         return poseInitialized;
// //     }

// //     public void resetPoseInitialization() {
// //         poseInitialized = false;
// //     }
// // }

// // package frc.robot.utils;

// // import frc.robot.Variables;
// // import frc.robot.subsystems.DriveSubsystem;

// // public class PoseManager {

// //     private final DriveSubsystem drive;
// //     private static final double MIN_UPDATE_DISTANCE = 0.1;

// //     private boolean poseInitialized = false;

// //     public PoseManager(DriveSubsystem drive) {
// //         this.drive = drive;
// //     }

// //     public void update() {
// //         if (!Variables.limelight.hasTarget) return;

// //         // Startup init from Limelight
// //         if (!poseInitialized && Variables.limelight.tagCount >= 1) {
// //             drive.setGyroYaw(Variables.limelight.ll_rot);

// //             drive.setPose(new Pose(
// //                 Variables.limelight.ll_x,
// //                 Variables.limelight.ll_y,
// //                 Variables.limelight.ll_rot
// //             ));

// //             poseInitialized = true;
// //             return;
// //         }

// //         // Normal snap updates after startup
// //         Pose current = drive.getPose();

// //         double dx = Variables.limelight.ll_x - current.getX();
// //         double dy = Variables.limelight.ll_y - current.getY();

// //         double dist = Math.hypot(dx, dy);

// //         if (dist > MIN_UPDATE_DISTANCE) {
// //             drive.setPose(new Pose(
// //                 Variables.limelight.ll_x,
// //                 Variables.limelight.ll_y,
// //                 Variables.drive.heading
// //             ));
// //         }
// //     }
// // }