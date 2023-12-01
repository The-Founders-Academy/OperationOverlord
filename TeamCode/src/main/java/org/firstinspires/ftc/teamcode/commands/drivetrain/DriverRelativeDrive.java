package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

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
        // Y is forward because the y axis points away from the driver station
        double leftRightMetersPerSecond = m_driver.getLeftX() * Constants.DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double forwardBackMetersPerSecond = m_driver.getLeftY() * Constants.DrivetrainConstants.MaxRobotSpeedMetersPerSecond;
        double omegaRadiansPerSecond = m_driver.getRightX() * Constants.DrivetrainConstants.MaxAngularVeloityRadiansPerSecond;
        m_drivetrain.moveFieldRelative(forwardBackMetersPerSecond, leftRightMetersPerSecond, omegaRadiansPerSecond);
    }
}
