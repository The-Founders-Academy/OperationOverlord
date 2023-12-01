package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
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
    private Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();


    // These values need to be tuned when we have access to the drivetrain.
    private PIDFController m_xPIDF = new PIDFController(0.1, 0, 0, 0);
    private PIDFController m_yPIDF = new PIDFController(0.1, 0 ,0, 0);
    private PIDFController m_angleRadiansPIDF = new PIDFController(0.1, 0.05, 0, 0);

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

        // Set PID error tolerances. If the error is less than the tolerance, the PID loop is considered finished
        setTranslationTolerance(DrivetrainConstants.TranslationToleranceMeters);
        m_angleRadiansPIDF.setTolerance(DrivetrainConstants.AnglePIDTolerance.getRadians());

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
    public void moveFieldRelative(double vXPercent, double vYPercent, double omegaPercent) {
        double vXMps = vXPercent * DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double vYMps = vYPercent * DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double omegaRps = omegaPercent * DrivetrainConstants.MaxAngularVeloityRadiansPerSecond;
        ChassisSpeeds speeds;

        if(DriverStation.getInstance().alliance == Alliance.BLUE) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getPose().getRotation().minus(new Rotation2d(Math.PI)));
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vXMps, vYMps, omegaRps, getPose().getRotation());
        }

        move(speeds);
    }

    @Override
    public void periodic() {
        updatePose();
        tunePIDs();

        dashboardTelemetry.addData("robotPoseX", getPose().getX());
        dashboardTelemetry.addData("robotPoseY", getPose().getY());
        dashboardTelemetry.addData("RobotPoseRotationDegrees", getPose().getRotation().getDegrees());
        dashboardTelemetry.addData("RobotHeadingDegrees", getHeading().getDegrees());
    }

    /**
     * This function does not return the absolute rotational position of the robot. It only returns how far the robot has rotated since it started or the last
     * resetHeading() call.
     * Boundaries: -360 deg or -PI rad to +360 deg or +PI rad
     * @return The current heading of the robot
     */
    public Rotation2d getHeading() {
        return new Rotation2d(m_imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
    }

    public void setTarget(Pose2d target) {
        m_xPIDF.setSetPoint(target.getX());
        m_yPIDF.setSetPoint(target.getY());
        m_angleRadiansPIDF.setSetPoint(target.getRotation().getRadians());
    }

    public void resetPIDs() {
        m_xPIDF.reset();
        m_yPIDF.reset();
        m_angleRadiansPIDF.reset();
    }

    public void moveToTarget() {
        double xPercent = UtilFunctions.clamp(m_xPIDF.calculate(getPose().getX()), -1, 1);
        double yPercent = UtilFunctions.clamp(m_yPIDF.calculate(getPose().getY()), -1, 1);
        double omegaPercent = UtilFunctions.clamp(m_angleRadiansPIDF.calculate(getPose().getRotation().getRadians()), -1, 1);
        moveFieldRelative(xPercent, yPercent, omegaPercent);
    }

    public boolean atTarget() {
        boolean atTranslation = (m_xPIDF.atSetPoint() && m_yPIDF.atSetPoint()); // True only if x and y are within tolerable distances from the set point
        boolean atRotation = m_angleRadiansPIDF.atSetPoint();

        if(atTranslation == true && atRotation == true) {
            return true;
        } else {
            return false;
        }
    }

    public void setTranslationTolerance(double tolerance) {
        m_xPIDF.setTolerance(tolerance);
        m_yPIDF.setTolerance(tolerance);
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

            m_pose = m_odometry.updateWithTime(m_elapsedTime.elapsedTime(), getPose().getRotation(), wheelSpeeds);
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

    private void tunePIDs() {
        m_xPIDF.setPIDF(DrivetrainConstants.XPIDF.p, DrivetrainConstants.XPIDF.i, DrivetrainConstants.XPIDF.d, DrivetrainConstants.XPIDF.f);
        m_xPIDF.setPIDF(DrivetrainConstants.YPIDF.p, DrivetrainConstants.YPIDF.i, DrivetrainConstants.YPIDF.d, DrivetrainConstants.YPIDF.f);
        m_xPIDF.setPIDF(DrivetrainConstants.AnglePIDF.p, DrivetrainConstants.AnglePIDF.i, DrivetrainConstants.AnglePIDF.d, DrivetrainConstants.AnglePIDF.f);
    }
}
