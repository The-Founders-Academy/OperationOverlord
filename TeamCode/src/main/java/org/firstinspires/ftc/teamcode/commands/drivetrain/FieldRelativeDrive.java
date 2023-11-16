package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class FieldRelativeDrive extends CommandBase {
    private Pose2d m_target;
    private Drivetrain m_drivetrain;

    public FieldRelativeDrive(Drivetrain drivetrain, GamepadEx driver) {
        m_target = new Pose2d(driver.getLeftX(), driver.getLeftY(), new Rotation2d(driver.getRightX()));
        m_drivetrain = drivetrain;

        addRequirements(m_drivetrain);
    }
}
