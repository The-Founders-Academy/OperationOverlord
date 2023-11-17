package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

public class DriveToPosition extends CommandBase {
    private Drivetrain m_drivetrain;
    private Pose2d m_targetPose;

    public DriveToPosition(Drivetrain drivetrain, Pose2d target) {
        m_drivetrain = drivetrain;
        m_targetPose = target;

        addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
        m_drivetrain.setTarget(m_targetPose);
    }

    @Override
    public void execute() {
        m_drivetrain.moveToTarget();
    }

    @Override
    public boolean isFinished() {
        return m_drivetrain.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.resetPIDs();
    }
}
