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
import org.firstinspires.ftc.teamcode.commands.drivetrain.ResetPose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.AllianceSingleton;

import java.util.function.Supplier;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadEx driver;
    private GamepadEx operator;

    private MecanumDrivetrain drivetrain;

    private Supplier<Double> leftXSupplier = new Supplier<Double>() {
        @Override
        public Double get() {
            return driver.getLeftX();
        };
    };

    private Supplier<Double> leftYSupplier = new Supplier<Double>() {
        @Override
        public Double get() {
            return -driver.getLeftY();
        };
    };

    private Supplier<Double> rightXSupplier = new Supplier<Double>() {
        @Override
        public Double get() {
            return driver.getButton(GamepadKeys.Button.DPAD_RIGHT) ? -20.0 : driver.getButton(GamepadKeys.Button.DPAD_LEFT) ? 20.0 : 0;
        };
    };
    private void driverControls() {
        drivetrain.setDefaultCommand(new DriverRelativeDrive(drivetrain, leftXSupplier, leftYSupplier, rightXSupplier));

        // When the driver presses the A button, drive forward 1 meter. We can use this to test odometry.
        driver.getGamepadButton(GamepadKeys.Button.A).whenPressed(new DriveToPosition(drivetrain, new Pose2d(1, 0, new Rotation2d(Math.PI)), 0.03));
        driver.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ResetPose(drivetrain));
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
        telemetry.update();
    }
}
