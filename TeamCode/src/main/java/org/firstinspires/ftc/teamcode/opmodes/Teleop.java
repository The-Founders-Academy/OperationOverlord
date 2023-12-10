package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm.ExtendLift;
import org.firstinspires.ftc.teamcode.commands.arm.Grip;
import org.firstinspires.ftc.teamcode.commands.arm.Release;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveToPosition;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.commands.drivetrain.ResetPose;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.arm.Claw;
import org.firstinspires.ftc.teamcode.subsystems.arm.Extender;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

@TeleOp(name="TeleOp")
public class Teleop extends CommandOpMode {

    private GamepadSubsystem m_driver;
    private GamepadSubsystem m_operator;

    private MecanumDrivetrain m_drivetrain;
    private Extender m_extender;
    private Claw m_claw;

    private MultipleTelemetry multiTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    private void driverControls() {
        m_drivetrain.setDefaultCommand(new DriverRelativeDrive(m_drivetrain, m_driver));

        // When the driver presses the A button, drive forward 1 meter. We can use this to test odometry.
        m_driver.buttonA().whenPressed(new DriveToPosition(m_drivetrain, new Pose2d(1, 0, new Rotation2d(0)), 0.03));
        m_driver.buttonB().whenPressed(new ResetPose(m_drivetrain));

        m_driver.bumperLeft().whenPressed(new Release(m_claw));
        m_driver.bumperRight().whenPressed(new Grip(m_claw));
        // Shoot airplane
    }

    private void operatorControls() {
        m_operator.setDefaultCommand(new ExtendLift(m_extender, m_operator));
        // Intake
    }

    @Override
    public void initialize() {
        setupDriverStation(); // This should always be called first

        // Controllers
        m_driver = new GamepadSubsystem(new GamepadEx(gamepad1));
        m_operator = new GamepadSubsystem(new GamepadEx(gamepad2));

        // Arm subsystems
        m_extender = new Extender();
        m_claw = new Claw("leftClaw", "rightClaw");

        // Drivetrain subsystems
        m_drivetrain = new MecanumDrivetrain(null, hardwareMap, "fL", "fR", "bL", "bR");

        // Initialize subsystems that need to be initialized
        m_claw.initialize();

        // These always get called last
        driverControls();
        operatorControls();

    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        multiTelemetry.update();
    }

    private void setupDriverStation() {
        DriverStation.getInstance().telemetry = telemetry;
        DriverStation.getInstance().setHardwareMap(hardwareMap);
        if(DriverStation.getInstance().alliance == DriverStation.Alliance.NONE) DriverStation.getInstance().alliance = DriverStation.Alliance.RED;
    }
}
