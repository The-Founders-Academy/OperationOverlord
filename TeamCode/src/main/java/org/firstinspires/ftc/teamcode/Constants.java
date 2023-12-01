package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

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
        public static double MaxRobotSpeedMetersPerSecond = 1.5; // Theoretical value from the strafer chassis product page
        public static double MaxAngularVeloityRadiansPerSecond = 2*Math.PI;
        public static Rotation2d AnglePIDTolerance = Rotation2d.fromDegrees(3.0); // Maximum 3 degree difference between set and actual
        public static Translation2d FrontLeftMotorLocation = new Translation2d(0.207, 0.168);
        public static Translation2d FrontRightMotorLocation = new Translation2d(0.207, -0.168);
        public static Translation2d BackLeftMotorLocation = new Translation2d(-0.207, 0.168);
        public static Translation2d BackRightMotorLocation = new Translation2d(-0.207, -0.168);
        public static double TranslationToleranceMeters = 0.03; // 3 millimeters of permissible error. Tune later if needed
        public static PIDFCoefficients XPIDF = new PIDFCoefficients(0,0,0,0);
        public static PIDFCoefficients YPIDF = new PIDFCoefficients(0,0,0,0);
        public static PIDFCoefficients AnglePIDF = new PIDFCoefficients(0,0,0,0);
        public static double DistancePerEncoderTick = 0.00056; // 0.56 mm per pulse
        public static final String FrontLeftName = "fL";
        public static final String FrontRightName = "fR";
        public static final String BackLeftName = "bL";
        public static final String BackRightName = "bR";
    }
}
