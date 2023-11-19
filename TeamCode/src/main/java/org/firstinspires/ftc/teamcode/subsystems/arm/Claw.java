package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;

public class Claw extends SubsystemBase {
    private ServoEx m_leftServo;
    private ServoEx m_rightServo;
    public Claw(ServoEx left, ServoEx right) {
        m_leftServo = left;
        m_rightServo = right;
    }

    public void grip() {
    }

    public void open() {

    }

    public void defaultPose() {

    }



}
