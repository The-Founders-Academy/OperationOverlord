package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends SubsystemBase {
    private MotorEx leftExtender, rightExtender;

    public Arm(HardwareMap hardwareMap, String leftExtenderName, String rightExtenderName) {
        leftExtender = new MotorEx(hardwareMap, leftExtenderName, Motor.GoBILDA.RPM_312);
        leftExtender.setRunMode(Motor.RunMode.PositionControl);
        leftExtender.setInverted(true);

        rightExtender = new MotorEx(hardwareMap, rightExtenderName, Motor.GoBILDA.RPM_312);
        rightExtender.setRunMode(Motor.RunMode.PositionControl);
    }

    public void moveToPosition(double position) {
        leftExtender.set(position);
        rightExtender.set(position);
    }


}
