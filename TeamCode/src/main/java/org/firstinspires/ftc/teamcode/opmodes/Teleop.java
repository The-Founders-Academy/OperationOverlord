package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.FieldRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drivetrain;
    private Motor fL, fR, bL, bR;

    private void driverControls() {
        // Driver
        drivetrain.setDefaultCommand(new FieldRelativeDrive(drivetrain, driver));

    }

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        drivetrain = new Drivetrain(new MecanumDrive(fL, fR, bL, bR), null);
        driverControls();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }
}
