package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private MotorEx leftExtender, rightExtender;
    private Encoder m_leftEncoder, m_rightEncoder;
    private PIDFController m_extenderPIDF = new PIDFController(0, 0, 0, 0);

    public Arm(MotorEx left, MotorEx right) {
        leftExtender = left;
        rightExtender = right;
    }

    public void move(double speed) {

    }

    public void moveToTarget() {
    }


}
