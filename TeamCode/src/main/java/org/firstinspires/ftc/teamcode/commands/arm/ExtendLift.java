package org.firstinspires.ftc.teamcode.commands.arm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.Extender;

public class ExtendLift extends CommandBase {
    private Extender m_extender;
    private GamepadSubsystem m_operator;

    public ExtendLift(Extender extender, GamepadSubsystem operator) {
        m_extender = extender;
        m_operator = operator;

        addRequirements(m_operator, m_extender);
    }

    @Override
    public void execute() {
        m_extender.extend(m_operator.getLeftY());
    }

}
