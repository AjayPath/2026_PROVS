package frc.robot;

public class Variables {

     public static class intake {
        public static double intakeRPS = 0;
    }   
    public static class feeder {
        public static double feederRPS = 0;
    }

    public static class floor {
        public static double floorRPS = 0;
    }

    public static class shooter {
        public static double shooterRPS = 0;
    }

    public static class pivot {
        public static double pivotPosition = 0;
    }
    
    public static class limelight {
        public static double tid = 0;
        public static double tx = 0;
        public static double ty = 0;

        public static double ll_x = 0;
        public static double ll_y = 0;
        public static double ll_rot = 0;

        public static boolean hasTarget = false;
        public static double tagCount = 0;

        public static double latencyMs = 0;
    }

    public static class drive {
        public static double heading = 0;   // degrees, same convention as drivetrain
        public static double turnRate = 0;  // deg/sec
        public static double currentX = 0;
        public static double currentY = 0;

        public static double visionYaw = 0;
        public static double targetHubAngleDeg = 0;
        public static double targetHubAngleErrorDeg = 0;
    }

    public static double distanceMeters = 0;
    public static double turnAngle = 0;
    public static double shooterRPS = 0;
    public static double middleRPS = 0;
}
