package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class Constants {
    public static class DrivetrainConstants {
        public static final double MaxRobotSpeedMetersPerSecond = 3;
        public static final double MaxAngularVeloityRadiansPerSecond = Math.PI;
        public static final double MaxTranslationError = 0; // CHANGE THIS
        public static final double MaxRotationError = 0; // CHANGE THIS
        public static final Rotation2d AnglePIDTolerance = Rotation2d.fromDegrees(3.0);
        public static final Translation2d FrontLeftMotorLocation = new Translation2d(0, 0);
        public static final Translation2d FrontRightMotorLocation = new Translation2d(0, 0);
        public static final Translation2d BackLeftMotorLocation = new Translation2d(0, 0);
        public static final Translation2d BackRightMotorLocation = new Translation2d(0, 0);
        public static final double TranslationToleranceMeters = 0.03; // 3 millimeters of permissible error. Tune later if needed
    }

    public static class OdometryConstants {
        public static final double TrackWidthInches = 0; // Distance between left odometry pod and right odometry pod
        public static final double CenterOdometryPodOffsetInches = 0; // Distance between center of rotation and center odometry pod
        public static final double OdometryPodTicksPerRevolution = 2000; // Obtained from the GoBILDA odometry pod webapge.
        public static final double OdometryWheelDiameterInhes = 1.89; // Obtained from the GoBILDA odometry pod webapge.
        public static final double DistancePerPulse = Math.PI * OdometryWheelDiameterInhes / OdometryPodTicksPerRevolution;
    }
}
