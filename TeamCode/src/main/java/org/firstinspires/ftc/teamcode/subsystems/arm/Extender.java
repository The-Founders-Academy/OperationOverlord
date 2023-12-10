package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.Constants.ArmConstants;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

public class Extender extends SubsystemBase {
    private MotorEx m_left, m_right;

    public Extender() {
        m_left = new MotorEx(DriverStation.getInstance().getHardwareMap(), ArmConstants.ExtenderLeftName, Motor.GoBILDA.RPM_312);
        m_right = new MotorEx(DriverStation.getInstance().getHardwareMap(), ArmConstants.ExtenderRightName, Motor.GoBILDA.RPM_312);

        m_right.setInverted(true);
        m_left.setInverted(false);

        m_left.setRunMode(Motor.RunMode.RawPower);
        m_right.setRunMode(Motor.RunMode.RawPower);
    }

    public void extend(double power) {
        m_left.set(power);
        m_right.set(power);
    }
}
