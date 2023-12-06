package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriverRelativeDrive;
import org.firstinspires.ftc.teamcode.commands.drivetrain.Reorient;
import org.firstinspires.ftc.teamcode.subsystems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

@TeleOp(name="MecanumTeleOp")
public class Teleop extends CommandOpMode {

    private GamepadSubsystem m_driver;
    private GamepadSubsystem m_operator;

    private MecanumDrivetrain m_drivetrain;
    private Telemetry dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
    private void driverControls() {
        m_drivetrain.setDefaultCommand(new DriverRelativeDrive(m_drivetrain, m_driver));

        m_driver.buttonB().whenPressed(new Reorient(m_drivetrain));
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

        m_drivetrain = new MecanumDrivetrain(null, hardwareMap);
        driverControls();
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
        telemetry.update();
        dashboardTelemetry.update();

        AprilTagGameDatabase.getCurrentGameTagLibrary();
    }

    private void setupDriverStation() {
        DriverStation.getInstance().telemetry = telemetry;
        if(DriverStation.getInstance().alliance == DriverStation.Alliance.NONE) DriverStation.getInstance().alliance = DriverStation.Alliance.RED;

    }
}
