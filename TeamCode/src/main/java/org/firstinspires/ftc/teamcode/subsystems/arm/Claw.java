package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

public class Claw extends SubsystemBase {
    private ServoEx m_leftServo;
    private ServoEx m_rightServo;
    private MultipleTelemetry multiTelemetry = new MultipleTelemetry(DriverStation.getInstance().telemetry, FtcDashboard.getInstance().getTelemetry());

    public Claw(String leftName, String rightName) {
        m_leftServo = new SimpleServo(DriverStation.getInstance().getHardwareMap(), leftName, -100, 0);
        m_rightServo = new SimpleServo(DriverStation.getInstance().getHardwareMap(), rightName, 0, 100);
    }

    public void grip() {
        m_leftServo.turnToAngle(0);
        m_rightServo.turnToAngle(0);
    }

    public void open() {
        m_leftServo.turnToAngle(-90);
        m_rightServo.turnToAngle(90);
    }

    public void initialize() {
        open();
    }

    @Override
    public void periodic() {
        multiTelemetry.addData("clawLeftPosition", m_leftServo.getPosition());
        multiTelemetry.addData("clawRightPosition", m_rightServo.getPosition());
    }
}
