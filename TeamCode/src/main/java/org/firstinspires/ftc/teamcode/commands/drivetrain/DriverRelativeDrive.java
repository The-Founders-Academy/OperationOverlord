package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

import java.util.function.Supplier;

public class DriverRelativeDrive extends CommandBase {
    private GamepadSubsystem m_driver;
    private MecanumDrivetrain m_drivetrain;

    public DriverRelativeDrive(MecanumDrivetrain mecanumDrive, GamepadSubsystem driver) {
        m_driver = driver;
        m_drivetrain = mecanumDrive;
        addRequirements(m_drivetrain, m_driver); // This may be a point of failure and may need testing.
    }

    @Override
    public void execute() {
        double xVelocityMetersPerSecond = m_driver.getLeftX() * Constants.DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double yVelocityMetersPerSecond = m_driver.getLeftX() * Constants.DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double omegaRadiansPerSecond = m_driver.getRightX() * Constants.DrivetrainConstants.MaxAngularVeloityRadiansPerSecond;
        m_drivetrain.moveFieldRelative(yVelocityMetersPerSecond, xVelocityMetersPerSecond, omegaRadiansPerSecond);
    }
}
