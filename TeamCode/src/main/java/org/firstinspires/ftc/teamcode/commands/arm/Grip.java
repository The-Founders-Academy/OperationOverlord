package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.Claw;

public class Grip extends CommandBase {
    private Claw m_claw;

    public Grip(Claw claw) {
        m_claw = claw;
        addRequirements(m_claw);
    }

    @Override
    public void execute() {
        m_claw.grip();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
