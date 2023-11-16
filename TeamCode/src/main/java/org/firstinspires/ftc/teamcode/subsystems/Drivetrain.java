package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public class Drivetrain extends SubsystemBase {
    private MecanumDrive m_mecanumDrive;
    private Pose2d m_robotPose;

    public Drivetrain(MecanumDrive mecanumDrive, Pose2d currentPose) {
        m_mecanumDrive = mecanumDrive;

        // Check if the robot's pose is already recorded. If so take that value. If not, assume the robot is at (0,0) and is rotated 0 degrees.
        if(currentPose != null) {
            m_robotPose = currentPose;
        } else {
            m_robotPose = new Pose2d(0, 0, new Rotation2d(0));
        }
    }

    public void drive(ChassisSpeeds chasisSpeeds) {

    }

    public Pose2d getRobotPose() {
        return m_robotPose;
    }

}
