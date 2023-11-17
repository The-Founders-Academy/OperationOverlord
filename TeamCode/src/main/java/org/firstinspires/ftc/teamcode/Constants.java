package org.firstinspires.ftc.teamcode;

public class Constants {
    public static class DrivetrainConstants {
        public static final double MaxStrafeVelocityFeetPerSecond = 0;
        public static final double MaxForwardVelocityFeetPerSecond = 0;
        public static final double MaxAngularVeloityRadiansPerSecond = 0;
        public static final double MaxTranslationError = 0; // CHANGE THIS
        public static final double MaxRotationError = 0; // CHANGE THIS
    }

    public static class OdometryConstants {
        public static final double TrackWidthInches = 0; // Distance between left odometry pod and right odometry pod
        public static final double CenterOdometryPodOffsetInches = 0; // Distance between center of rotation and center odometry pod
        public static final double OdometryPodTicksPerRevolution = 2000; // Obtained from the GoBILDA odometry pod webapge.
        public static final double OdometryWheelDiameterInhes = 1.89; // Obtained from the GoBILDA odometry pod webapge.
        public static final double DistancePerPulse = Math.PI * OdometryWheelDiameterInhes / OdometryPodTicksPerRevolution;
    }
}
