package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.Claw;

public class Release extends CommandBase {
    private Claw m_claw;

    public Release(Claw claw) {
        m_claw = claw;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.open();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
