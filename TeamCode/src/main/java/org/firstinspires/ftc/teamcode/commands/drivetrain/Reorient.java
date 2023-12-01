package org.firstinspires.ftc.teamcode.commands.drivetrain;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;

public class Reorient extends CommandBase {
    private MecanumDrivetrain m_drivetrain;

    public Reorient(MecanumDrivetrain drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.reorient();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
