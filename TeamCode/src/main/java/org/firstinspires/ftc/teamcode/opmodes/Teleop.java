package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveToPosition;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.AllianceSingleton;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;

    private MecanumDrivetrain drivetrain;

    private void driverControls() {
        drivetrain.setDefaultCommand(new DriverRelativeDrive(drivetrain, driver));

        // When the driver presses the A button, drive forward 1 meter. We can use this to test odometry.
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new DriveToPosition(drivetrain, new Pose2d(3, 0, new Rotation2d(0)), 0.03));
        // Score
        // Shoot airplane
    }

    private void operatorControls() {
        // Move arm to pose
        // Intake
    }

    @Override
    public void initialize() {
        AllianceSingleton.getInstance().setAlliance(AllianceSingleton.Alliance.BLUE);
        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL", "fR", "bL", "bR", telemetry);
        driverControls();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.addData("gamepadX", driver.getLeftX());
        telemetry.addData("gamepadY", driver.getLeftY());
        telemetry.update();
    }
}
