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
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

import java.sql.Driver;
import java.util.function.Supplier;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadSubsystem m_driver;
    private GamepadSubsystem m_operator;

    private MecanumDrivetrain m_drivetrain;

    private void driverControls() {
        m_drivetrain.setDefaultCommand(new DriverRelativeDrive(m_drivetrain, m_driver));

        // When the driver presses the A button, drive forward 1 meter. We can use this to test odometry.
        m_driver.buttonA().whenPressed(new DriveToPosition(m_drivetrain, new Pose2d(1, 0, new Rotation2d(Math.PI)), 0.03));
        m_driver.buttonB().whenPressed(new ResetPose(m_drivetrain));
        // Score
        // Shoot airplane
    }

    private void operatorControls() {
        // Move arm to pose
        // Intake
    }

    @Override
    public void initialize() {
        setupDriverStation();

        m_driver = new GamepadSubsystem(new GamepadEx(gamepad1));
        m_operator = new GamepadSubsystem(new GamepadEx(gamepad2));

        m_drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL", "fR", "bL", "bR");
        driverControls();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.update();
    }

    private void setupDriverStation() {
        DriverStation.getInstance().telemetry = telemetry;
        if(DriverStation.getInstance().alliance == DriverStation.Alliance.NONE) DriverStation.getInstance().alliance = DriverStation.Alliance.BLUE;
    }
}
