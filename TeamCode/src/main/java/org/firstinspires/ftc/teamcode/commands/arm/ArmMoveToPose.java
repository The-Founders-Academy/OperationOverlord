package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;

public class ArmMoveToPose extends CommandBase {
    private Arm m_arm;
    private double m_extenderLengthMeters;

    public ArmMoveToPose(Arm arm, double extenderLengthMeters) {
        m_arm = arm;
        m_extenderLengthMeters =extenderLengthMeters;

        addRequirements(m_arm);
    }

    @Override
    public void execute() {
        m_arm.moveToPosition(m_extenderLengthMeters);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }


}
