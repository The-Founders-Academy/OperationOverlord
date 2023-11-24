package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import androidx.core.math.MathUtils;

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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.UtilFunctions;
import org.firstinspires.ftc.teamcode.utility.AllianceSingleton;

public class MecanumDrivetrain extends SubsystemBase {
    private MecanumMotor m_frontLeft, m_frontRight, m_backLeft, m_backRight;
    private MecanumDriveKinematics m_kinematics;
    private MecanumDriveOdometry m_odometry;
    private Pose2d m_pose;
    private IMU m_imu;
    private Timer m_elapsedTime;
    private Vision m_vision;

    // These values need to be tuned when we have access to the drivetrain.
    private PIDFController m_xPIDF = new PIDFController(1, 0, 0, 0);
    private PIDFController m_yPIDF = new PIDFController(1, 0 ,0, 0);
    private PIDFController m_angleRadiansPIDF = new PIDFController(1, 0, 0, 0);

    public MecanumDrivetrain(Pose2d initialPose, HardwareMap hardwareMap, String frontLeftName, String frontRightName, String backLeftName, String backRightName) {
        // Initialize hardware
        m_frontLeft = new MecanumMotor(new MotorEx(hardwareMap, frontLeftName, Motor.GoBILDA.RPM_312));
        m_frontRight = new MecanumMotor(new MotorEx(hardwareMap, frontRightName, Motor.GoBILDA.RPM_312));
        m_backLeft = new MecanumMotor(new MotorEx(hardwareMap, backLeftName, Motor.GoBILDA.RPM_312));
        m_backRight = new MecanumMotor(new MotorEx(hardwareMap, backRightName, Motor.GoBILDA.RPM_312));
        m_imu = hardwareMap.get(IMU.class, "imu");
        m_imu.initialize(
                new IMU.Parameters(
                        // TO-DO: When we actually mount the control hub, we will need to know the actual values for this
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                        )
                )
        );

        // Check if we have a non-null starting pose
        if(initialPose != null) {
            m_pose = initialPose;
        } else {
            m_pose = new Pose2d(); // Pose at (0,0,0)
        }

        // Set

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
        chassisSpeeds.vxMetersPerSecond = MathUtils.clamp(chassisSpeeds.vxMetersPerSecond, -1, 1) * DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        chassisSpeeds.vyMetersPerSecond = MathUtils.clamp(chassisSpeeds.vyMetersPerSecond, -1, 1) * DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        chassisSpeeds.omegaRadiansPerSecond = MathUtils.clamp(chassisSpeeds.omegaRadiansPerSecond, -1, 1) * DrivetrainConstants.MaxAngularVeloityRadiansPerSecond;

        MecanumDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(chassisSpeeds);
        m_frontLeft.setTargetVelocity(wheelSpeeds.frontLeftMetersPerSecond);
        m_frontRight.setTargetVelocity(wheelSpeeds.frontRightMetersPerSecond);
        m_backLeft.setTargetVelocity(wheelSpeeds.rearLeftMetersPerSecond);
        m_backRight.setTargetVelocity(wheelSpeeds.rearRightMetersPerSecond);
    }


    public void moveFieldRelative(double velocityXMetersPerSecond, double velocityYMetersPerSecond, double omegaRadiansPerSecond) {
        ChassisSpeeds speeds;
        if(AllianceSingleton.getInstance().getAlliance() == AllianceSingleton.Alliance.RED) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocityXMetersPerSecond, velocityYMetersPerSecond, omegaRadiansPerSecond, getHeading().minus(new Rotation2d(Math.PI)));
        } else {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(velocityXMetersPerSecond, velocityYMetersPerSecond, omegaRadiansPerSecond, getHeading());
        }
        move(speeds);
    }

    @Override
    public void periodic() {
        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(
                m_frontLeft.getVelocity(), m_frontRight.getVelocity(),
                m_backLeft.getVelocity(), m_backRight.getVelocity()
        );

        m_pose = m_odometry.updateWithTime(m_elapsedTime.elapsedTime(), getHeading(), wheelSpeeds);
    }

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
        // The values returned from calculate will cause the wheels to spin at maximum speed unless we're within less than sqrt(2) meters from the target position.
        // If we used a smoothing function like tanh, we could get smoother movement so that the robot slows down smoothly as it approaches its target.
        double x = UtilFunctions.clamp(m_xPIDF.calculate(getPose().getX()), -1, 1); // This may need to be altered to prevent overpowering the motors
        double y = UtilFunctions.clamp(m_yPIDF.calculate(getPose().getY()), -1, 1); // This may need to be altered to prevent overpowering the motors
        double omegaRadiansPerSecond = UtilFunctions.clamp(m_angleRadiansPIDF.calculate(getHeading().getRadians()), -1, 1); // This may need to be altered to prevent overpowering the motors
        moveFieldRelative(x, y, omegaRadiansPerSecond);
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
}
