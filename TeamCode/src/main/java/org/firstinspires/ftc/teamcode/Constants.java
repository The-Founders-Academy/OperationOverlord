package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class Constants {

    /*
    Wheel center distances:
    ----------------------
    8 hole crossbar:
    horizontal: 178 mm
    vertical: 168 mm

    10 hole crossbar (what we'll have on Monday/Tuesday):
    horizontal: 207 mm
    vertical: 168 mm
     */
    public static class DrivetrainConstants {
        public static final double MaxRobotSpeedMetersPerSecond = 3;
        public static final double MaxAngularVeloityRadiansPerSecond = Math.PI;
        public static final Rotation2d AnglePIDTolerance = Rotation2d.fromDegrees(3.0); // Maximum 3 degree difference between set and actual
        public static final Translation2d FrontLeftMotorLocation = new Translation2d(0.178, 0.178);
        public static final Translation2d FrontRightMotorLocation = new Translation2d(0.178, -0.178);
        public static final Translation2d BackLeftMotorLocation = new Translation2d(-0.178, 0.178);
        public static final Translation2d BackRightMotorLocation = new Translation2d(-0.178, -0.178);
        public static final double TranslationToleranceMeters = 0.03; // 3 millimeters of permissible error. Tune later if needed
    }

}
