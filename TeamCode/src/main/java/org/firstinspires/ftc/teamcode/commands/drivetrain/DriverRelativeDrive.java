package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriverRelativeDrive extends CommandBase {

    private double vX;
    private double vY;
    private double omegaRadiansPerSecond;
    private Drivetrain m_drivetrain;

    public DriverRelativeDrive(Drivetrain drivetrain, GamepadEx driver) {
        vX = driver.getLeftY();
        vY = driver.getLeftX();
        omegaRadiansPerSecond = driver.getRightX();

        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain); // This may be a point of failure and may need testing.
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(vX * Constants.DrivetrainConstants.MaxLinearVelocityMetersPerSecond, vY * Constants.DrivetrainConstants.MaxLinearVelocityMetersPerSecond, omegaRadiansPerSecond * Constants.DrivetrainConstants.MaxAngularVeloityRadiansPerSecond, m_drivetrain.getRobotPose().getRotation());
        m_drivetrain.drive(chassisSpeeds);
    }
}
