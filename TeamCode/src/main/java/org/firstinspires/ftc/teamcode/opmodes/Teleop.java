package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;

    private Drivetrain drivetrain;
    private Motor fL, fR, bL, bR;

    private void driverControls() {
        drivetrain.setDefaultCommand(new DriverRelativeDrive(drivetrain, driver));
        // Score / release game piece
    }

    private void operatorControls() {
        // Move arm to pose
        // Intake
    }

    @Override
    public void initialize() {
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        fL = new Motor(hardwareMap, "fL");
        fR = new Motor(hardwareMap, "fR");
        bL = new Motor(hardwareMap, "bL");
        bR = new Motor(hardwareMap, "bR");

        drivetrain = new Drivetrain(fL, fR, bL, bR, null);
        driverControls();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        
        //Telemtry
        //telemetry.addData("robotPoseX", Drivetrain.getRobotPoseX());
        //telemetry.addData("robotPoseY", Drivetrain.getRobotPoseY());
        //telemetry.addData("headingDegrees", Drivetrain.getRobotPoseDegrees);
        //telemetry.update();

        // Telemetry Update Code listed above in comment form
    }
}
