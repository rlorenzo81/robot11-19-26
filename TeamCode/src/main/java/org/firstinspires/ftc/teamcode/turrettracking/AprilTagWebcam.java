
package org.firstinspires.ftc.teamcode.turrettracking;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagWebcam {

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private AprilTagLibrary tagLibrary;

    private List<AprilTagDetection> detectedTags =
            new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        // Includes the BIOBUZZ AprilTag clusters.
        tagLibrary =
                AprilTagGameDatabase.getCurrentGameTagLibrary();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(
                        DistanceUnit.CM,
                        AngleUnit.DEGREES)
                .setTagLibrary(tagLibrary)
                .build();

        VisionPortal.Builder builder =
                new VisionPortal.Builder();

        builder.setCamera(
                hwMap.get(WebcamName.class, "Webcam 1"));

        builder.setCameraResolution(new Size(640, 480));

        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {

        if (aprilTagProcessor != null) {
            detectedTags = aprilTagProcessor.getDetections();
        }
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(
            AprilTagDetection detection) {

        if (detection == null || telemetry == null) {
            return;
        }

        // Individual AprilTag detection
        if (detection instanceof AprilTagSingleDetection) {

            AprilTagSingleDetection singleDet =
                    (AprilTagSingleDetection) detection;

            if (singleDet.metadata != null) {

                telemetry.addLine(String.format(
                        "\n==== (ID %d) %s",
                        singleDet.id,
                        singleDet.metadata.name));

                displayPoseTelemetry(detection);

            } else {

                telemetry.addLine(String.format(
                        "\n==== (ID %d) Unknown",
                        singleDet.id));

                if (singleDet.center != null) {
                    telemetry.addLine(String.format(
                            "Center %6.0f %6.0f (pixels)",
                            singleDet.center.x,
                            singleDet.center.y));
                }
            }

            // AprilTag cluster detection (BIOBUZZ)
        } else if (detection instanceof AprilTagClusterDetection) {

            AprilTagClusterDetection clusterDet =
                    (AprilTagClusterDetection) detection;

            telemetry.addLine(String.format(
                    "\n==== Cluster: %s",
                    clusterDet.metadata.name));

            telemetry.addData(
                    "Cluster Visibility",
                    clusterDet.percentClusterFound + "%");

            displayPoseTelemetry(detection);
        }
    }

    private void displayPoseTelemetry(
            AprilTagDetection detection) {

        if (detection.ftcPose == null) {
            telemetry.addLine("Pose unavailable");
            return;
        }

        telemetry.addLine(String.format(
                "XYZ %6.1f %6.1f %6.1f (cm)",
                detection.ftcPose.x,
                detection.ftcPose.y,
                detection.ftcPose.z));

        telemetry.addLine(String.format(
                "PRY %6.1f %6.1f %6.1f (deg)",
                detection.ftcPose.pitch,
                detection.ftcPose.roll,
                detection.ftcPose.yaw));

        telemetry.addLine(String.format(
                "RBE %6.1f %6.1f %6.1f (cm, deg, deg)",
                detection.ftcPose.range,
                detection.ftcPose.bearing,
                detection.ftcPose.elevation));
    }

    // Works for standalone tags and BIOBUZZ cluster-member IDs.
    public AprilTagDetection getTagBySpecificId(int id) {

        // First check standalone AprilTags.
        for (AprilTagDetection detection : detectedTags) {

            if (detection instanceof AprilTagSingleDetection) {

                AprilTagSingleDetection singleDet =
                        (AprilTagSingleDetection) detection;

                if (singleDet.id == id) {
                    return singleDet;
                }
            }
        }

        // A BIOBUZZ tag ID belongs to a cluster.
        // Look up its cluster, then find the visible detection.
        if (tagLibrary != null) {

            AprilTagClusterMetadata wantedCluster =
                    tagLibrary.lookupCluster(id);

            if (wantedCluster != null) {

                for (AprilTagDetection detection : detectedTags) {

                    if (detection instanceof AprilTagClusterDetection) {

                        AprilTagClusterDetection clusterDet =
                                (AprilTagClusterDetection) detection;

                        if (clusterDet.metadata.name.equals(
                                wantedCluster.name)) {

                            return clusterDet;
                        }
                    }
                }
            }
        }

        return null;
    }

    // Find a currently visible cluster by name.
    public AprilTagClusterDetection getClusterByName(
            String name) {

        if (name == null) {
            return null;
        }

        for (AprilTagDetection detection : detectedTags) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection clusterDet =
                        (AprilTagClusterDetection) detection;

                if (clusterDet.metadata.name.equalsIgnoreCase(name)) {
                    return clusterDet;
                }
            }
        }

        return null;
    }

    // Find a visible cluster on the requested alliance.
    public AprilTagClusterDetection getAllianceCluster(
            boolean redAlliance) {

        String prefix = redAlliance ? "R" : "B";

        for (AprilTagDetection detection : detectedTags) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection clusterDet =
                        (AprilTagClusterDetection) detection;

                if (clusterDet.metadata.name
                        .toUpperCase().startsWith(prefix)) {

                    return clusterDet;
                }
            }
        }

        return null;
    }

    public void stop() {

        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        aprilTagProcessor = null;
        detectedTags = new ArrayList<>();
    }
}
