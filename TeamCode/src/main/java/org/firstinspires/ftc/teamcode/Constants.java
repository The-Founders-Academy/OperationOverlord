package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
    @Config
    public static class DrivetrainConstants {
        public static final double MaxRobotSpeedMetersPerSecond = 1.5; // Theoretical value from the strafer chassis product page
        public static final double MaxAngularVeloityRadiansPerSecond = 2*Math.PI;
        public static final Rotation2d AnglePIDTolerance = Rotation2d.fromDegrees(3.0); // Maximum 3 degree difference between set and actual
        public static final Translation2d FrontLeftMotorLocation = new Translation2d(0.178, 0.168);
        public static final Translation2d FrontRightMotorLocation = new Translation2d(0.178, -0.168);
        public static final Translation2d BackLeftMotorLocation = new Translation2d(-0.178, 0.168);
        public static final Translation2d BackRightMotorLocation = new Translation2d(-0.178, -0.168);
        public static final double TranslationToleranceMeters = 0.03; // 3 millimeters of permissible error. Tune later if needed
        public static final double DistancePerEncoderTick = 0.00056; // 0.56 mm per pulse
        public static PIDFCoefficients xCoefficients = new PIDFCoefficients(0.5, 0.3, 0, 0);
        public static PIDFCoefficients yCoefficients = new PIDFCoefficients(0.5, 0.3, 0, 0);
        public static PIDFCoefficients AngleCoefficients = new PIDFCoefficients(0.5, 0, 0, 0);
        public static double TestX = 0;
        public static double TestY = 0;
        public static double TestDegrees = 0;
    }

}
