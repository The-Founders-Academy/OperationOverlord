package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import android.util.Size;

import com.acmerobotics.dashboard.canvas.Rotation;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * This class encapsulates all cameras on the robot and the processing of the images they produce.
 */
public class Vision {
    private AprilTagProcessor m_tagProcessor;
    private VisionPortal m_visionPortal;

    public Vision(HardwareMap hardwareMap) {
        // Custom configuration for the AprilTagProcessor via a builder
        AprilTagProcessor.Builder processorBuilder = new AprilTagProcessor.Builder();
        processorBuilder.setDrawTagOutline(true);
        processorBuilder.setDrawTagID(true);
        processorBuilder.setDrawAxes(true);
        processorBuilder.setOutputUnits(DistanceUnit.METER, AngleUnit.RADIANS);
        processorBuilder.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary());

        // Build the AprilTagProcessor
        m_tagProcessor = processorBuilder.build();

        // Custom configuration for the vision portal via a builder
        VisionPortal.Builder visionPortalBuilder = new VisionPortal.Builder();
        visionPortalBuilder.addProcessor(m_tagProcessor);
        visionPortalBuilder.enableLiveView(true);
        visionPortalBuilder.setCameraResolution(new Size(1280, 720));
        visionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "frontCamera"));

        // Build the VisionPortal
        m_visionPortal = visionPortalBuilder.build();
    }

    public Pose2d getRobotPoseFromAprilTags() {
        if(m_tagProcessor.getDetections().isEmpty() == false) {
            AprilTagDetection detection = m_tagProcessor.getDetections().get(0);

            // For an understanding of this line, check the Google drive to see how april tag coordinates and FTCLib coordinates differ
            return new Pose2d(-detection.ftcPose.y, detection.ftcPose.x, new Rotation2d(detection.ftcPose.yaw).minus(Rotation2d.fromDegrees(90)));
        } else {
            return null;
        }
    }
}
