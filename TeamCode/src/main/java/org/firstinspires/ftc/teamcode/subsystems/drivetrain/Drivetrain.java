package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.arcrobotics.ftclib.kinematics.Odometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DrivetrainConstants;
import org.firstinspires.ftc.teamcode.Constants.OdometryConstants;
import org.firstinspires.ftc.teamcode.UtilFunctions;

public class Drivetrain extends SubsystemBase {
    private MecanumDrive m_mecanumDrive;
    private HolonomicOdometry m_odometry;
    private Encoder m_leftOdometer;
    private Encoder m_centerOdometer;
    private Encoder m_rightOdometer;

    // These values need to be tuned when we have access to the drivetrain.
    private PIDFController m_xPIDF = new PIDFController(0, 0, 0, 0);
    private PIDFController m_yPIDF = new PIDFController(0, 0 ,0, 0);
    private PIDFController m_angleRadiansPIDF = new PIDFController(0, 0, 0, 0);


    private Pose2d m_robotPose;


    public Drivetrain(Motor frontLeft, Motor frontRight, Motor backLeft, Motor backRight, Pose2d currentPose) {
        m_mecanumDrive = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Check if the robot's pose is already recorded. If so take that value. If not, assume the robot is at (0,0) and is rotated 0 degrees.
        if(currentPose != null) {
            m_robotPose = currentPose;
        } else {
            m_robotPose = new Pose2d(0, 0, new Rotation2d(0));
        }

        m_odometry.updatePose(m_robotPose);

        // We will be using the frontLeft, frontRight, and backLeft encoder ports for our odometry pods
        m_leftOdometer = frontLeft.encoder.setDistancePerPulse(OdometryConstants.DistancePerPulse);
        m_centerOdometer = frontRight.encoder.setDistancePerPulse(OdometryConstants.DistancePerPulse);
        m_rightOdometer = backLeft.encoder.setDistancePerPulse(OdometryConstants.DistancePerPulse);

        m_rightOdometer.setDirection(Motor.Direction.REVERSE); // We need to test this value to see whether the left motor is reversed or the right motor

        m_odometry = new HolonomicOdometry(
                m_leftOdometer::getDistance,
                m_rightOdometer::getDistance,
                m_centerOdometer::getDistance,
                OdometryConstants.TrackWidthInches,
                OdometryConstants.CenterOdometryPodOffsetInches
        );
    }

    public void moveFieldRelative(double strafeSpeed, double forwardSpeed, double omegaRadiansPerSecond) {
        // We will want to switch this to check if the speeds are under a certain threshold, since joystick drift will likely always give us some small values for each speed.
        if(strafeSpeed == 0 && forwardSpeed == 0 && omegaRadiansPerSecond == 0) {
            m_mecanumDrive.stop();
            return;
        }
        m_mecanumDrive.driveFieldCentric(strafeSpeed * DrivetrainConstants.MaxStrafeVelocityFeetPerSecond, forwardSpeed * DrivetrainConstants.MaxForwardVelocityFeetPerSecond, omegaRadiansPerSecond * DrivetrainConstants.MaxAngularVeloityRadiansPerSecond, m_robotPose.getRotation().getDegrees());
    }

    public Pose2d getRobotPose() {
        return m_robotPose;
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
        double x = UtilFunctions.clamp(m_xPIDF.calculate(getRobotPose().getX()), -1, 1); // This may need to be altered to prevent overpowering the motors
        double y = UtilFunctions.clamp(m_yPIDF.calculate(getRobotPose().getY()), -1, 1); // This may need to be altered to prevent overpowering the motors
        double omegaRadiansPerSecond = UtilFunctions.clamp(m_angleRadiansPIDF.calculate(m_robotPose.getRotation().getRadians()), -1, 1); // This may need to be altered to prevent overpowering the motors
        moveFieldRelative(x, y, omegaRadiansPerSecond);
    }

    public boolean atTarget() {
        boolean atTranslation = m_xPIDF.getPositionError() < DrivetrainConstants.MaxTranslationError && m_yPIDF.getPositionError() < DrivetrainConstants.MaxTranslationError;
        boolean atRotation = m_angleRadiansPIDF.getPositionError() < DrivetrainConstants.MaxRotationError;

        if(atTranslation == true && atRotation == true) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void periodic() {
        m_odometry.updatePose();
        m_robotPose = m_odometry.getPose();
    }

}
