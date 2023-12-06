package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.arcrobotics.ftclib.util.Timing.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.UtilFunctions;
import org.firstinspires.ftc.teamcode.utility.DriverStation;
import org.firstinspires.ftc.teamcode.utility.DriverStation.Alliance;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

// SMART DASHBOARD IP: 192.168.43.1:8080/dash
public class MecanumDrivetrain extends SubsystemBase {
    private MecanumMotor m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
    /*
    FIELD COORDINATE EXPLANATION:
    Positve Y - points away from the red driver station
    Positive X - points to the right of the red driver station
    Rotation - positive rotation turns CCW (to the left of the red driver station)

    The position and rotation in m_pose are in field coordinates and do not change with robot orientation.
     */
    private Pose2d m_pose;
    // The heading value produced by m_imu is relative to a starting position and is exclusively for robot control.
    private IMU m_imu;
    private Timer m_elapsedTime;
    // private Vision m_vision;
    private MultipleTelemetry telemetry = new MultipleTelemetry(DriverStation.getInstance().telemetry, FtcDashboard.getInstance().getTelemetry());

    public MecanumDrivetrain(Pose2d initialPose, HardwareMap hardwareMap) {
        // Initialize hardware

        m_frontLeft = new MecanumMotor(new MotorEx(hardwareMap, DrivetrainConstants.FrontLeftName, Motor.GoBILDA.RPM_312));
        m_frontRight = new MecanumMotor(new MotorEx(hardwareMap,DrivetrainConstants.FrontRightName, Motor.GoBILDA.RPM_312));
        m_backLeft = new MecanumMotor(new MotorEx(hardwareMap, DrivetrainConstants.BackLeftName, Motor.GoBILDA.RPM_312));
        m_backRight = new MecanumMotor(new MotorEx(hardwareMap, DrivetrainConstants.BackRightName, Motor.GoBILDA.RPM_312));

        m_backLeft.setInverted(true);
        m_frontLeft.setInverted(true);

        // m_vision = new Vision(hardwareMap);
        m_imu = hardwareMap.get(IMU.class, "imu");
        m_imu.initialize(
                new IMU.Parameters(
                        // TO-DO: When we actually mount the control hub, we will need to know the actual values for this
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                        )
                )
        );

        // Check if we have a non-null starting pose
        if(initialPose != null) {
            m_pose = initialPose;
        } else {
            m_pose = new Pose2d(); // Pose at (0,0,0)
        }

        // Initialize kinematics & odometry
        m_kinematics = new MecanumDriveKinematics(
                DrivetrainConstants.FrontLeftMotorLocation, DrivetrainConstants.FrontRightMotorLocation,
                DrivetrainConstants.BackLeftMotorLocation, DrivetrainConstants.BackRightMotorLocation
                );

        m_odometry = new MecanumDriveOdometry(
                m_kinematics, new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                m_pose
        );


        // We only need a timer object to call m_odometry.updateWithTime(), so the specific length doesn't matter, as long as it lasts longer than an FTC match.
        m_elapsedTime = new Timer(1200); // 20 minutes
        m_elapsedTime.start();
    }

    public Pose2d getPose() {
        return m_pose;
    }

    private void move(ChassisSpeeds chassisSpeeds) {
        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
        m_frontLeft.setTargetVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        m_frontRight.setTargetVelocity(wheelSpeeds.frontRightMetersPerSecond);
        m_backLeft.setTargetVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        m_backRight.setTargetVelocity(wheelSpeeds.rearRightMetersPerSecond);
    }

    // positive y = away from you
    // positive x = to your right
    public void moveFieldRelative(double vForwardPercent, double vRightPercent, double omegaPercent) {
        double vFwdMps = vForwardPercent * DrivetrainConstants.MaxRobotSpeedMetersPerSecond * 0.3;
        double vRtMps = vRightPercent * DrivetrainConstants.MaxRobotSpeedMetersPerSecond * 0.3;
        double omegaRps = -omegaPercent * DrivetrainConstants.MaxAngularVeloityRadiansPerSecond * 0.2;
        ChassisSpeeds speeds;

        if(DriverStation.getInstance().alliance == Alliance.BLUE) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vFwdMps, -vRtMps, omegaRps, getPose().getRotation().minus(new Rotation2d(Math.PI)));
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vFwdMps, -vRtMps, omegaRps, getPose().getRotation());
        }

        move(speeds);
    }

    @Override
    public void periodic() {
        updatePose();

        telemetry.addData("robotPoseX", getPose().getX());
        telemetry.addData("robotPoseY", getPose().getY());
        telemetry.addData("RobotPoseRotationRad", getPose().getRotation().getRadians());
        telemetry.addData("RobotHeadingDegrees", getHeading().getDegrees());
    }

    /**
     * This function does not return the absolute rotational position of the robot. It only returns how far the robot has rotated since it started or the last
     * resetHeading() call.
     * Boundaries: -180 deg or -PI rad to +180 deg or +PI rad
     * @return The current heading of the robot
     */
    public Rotation2d getHeading() {
        return new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    // This function is used by the mecanumDrivetrain class itself to update its position on the field. It works by
    // first asking if the vision object can read an april tag. If so, it will use the position data provided by that april tag. Otherwise,
    // It will use the odometry object to update the robot's postiion.
    private void updatePose() {
        Pose2d visionPose = null; // m_vision.getRobotPoseFromAprilTags();

        // Check to see if we saw and read an april tag
        if(visionPose != null) {
            m_pose = visionPose;
            m_odometry.resetPosition(m_pose, getHeading());
        } else {
            MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                    m_frontLeft.getVelocity(), m_frontRight.getVelocity(),
                    m_backLeft.getVelocity(), m_backRight.getVelocity()
            );

            Pose2d tempPose = m_odometry.updateWithTime(m_elapsedTime.elapsedTime(), getHeading(), wheelSpeeds);
            Pose2d realPose = new Pose2d(-tempPose.getY(), tempPose.getX(), tempPose.getRotation());
            m_pose = realPose;
        }
    }

    public void reorient() {
        m_imu.resetYaw();
        m_pose.getRotation().times(0);
        m_odometry.resetPosition(m_pose, m_pose.getRotation());
    }

    public void resetPose(Pose2d newPose) {
        m_pose = newPose;
    }

}
