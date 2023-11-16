package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;

    private MecanumDrive drivetrain;
    private Motor fL, fR, bL, bR;

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        drivetrain = new MecanumDrive(fL, fR, bL, bR);
    }

    @Override
    public void run() {

    }
}
