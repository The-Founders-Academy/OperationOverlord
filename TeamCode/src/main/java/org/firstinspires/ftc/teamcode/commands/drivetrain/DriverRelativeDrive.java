package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

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
        m_drivetrain.moveFieldRelative(vX, vY, omegaRadiansPerSecond);
    }
}