package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.drivetrain.DriveToPosition;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.utility.DriverStation;

@Autonomous(name="bFwdMeter")
public class BFwdMeter extends CommandOpMode {
    private MecanumDrivetrain m_drivetrain;
    private Pose2d initialPose = new Pose2d(0, -1, new Rotation2d(0));
    private Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0));
    @Override
    public void initialize() {
        DriverStation.getInstance().alliance = DriverStation.Alliance.BLUE;
        m_drivetrain = new MecanumDrivetrain(initialPose, hardwareMap);
        CommandScheduler.getInstance().schedule(new DriveToPosition(m_drivetrain, targetPose, Constants.DrivetrainConstants.TranslationToleranceMeters));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

}




