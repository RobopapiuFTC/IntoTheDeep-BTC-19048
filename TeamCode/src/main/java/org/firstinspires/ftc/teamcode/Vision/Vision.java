package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;


@Config
public class Vision {
    // Limelight and claw configuration
    public static double limelightHeight = 10; // Camera height in inches
    public static double limelightAngle = 60; // Camera angle (0° = down, 90° = forward)
    public static double intakeForwardOffset = 0; // Claw's forward offset from the camera
    public static double intakeLateralOffset = 3; // Claw's lateral (right is +) offset from the camera

    private Pose sample = new Pose(), difference = new Pose(), target = new Pose(); // The best sample's position
    private Pose cachedTarget = new Pose(); // Cached best sample
    public int intake = 0;
    private Limelight3A limelight;
    private PathChain toTarget;
    private LLResult result;
    private Telemetry telemetry;
    private String unwanted1;
    private String unwanted2;
    private double bestAngle;
    private Follower f;
    private int pipeline;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry, String unwanted1, String unwanted2, Follower f, int pipeline) {
        this.unwanted1 = unwanted1;
        this.unwanted2 = unwanted2;
        this.telemetry = telemetry;
        this.f = f;
        this.pipeline = pipeline;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        f.update();
        cachedTarget = f.getPose();
        f.update();
    }

    public void find() {
        result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();

        if (detections.isEmpty()) {
            telemetry.addData("Detections", "None");
            target = cachedTarget.copy();
            return;
        }

        // List to store scored detections
        List<LL3ADetection> scoredDetections = new ArrayList<>();

        for (LLResultTypes.DetectorResult detection : detections) {
            String c = detection.getClassName();

            boolean colorMatch = true;

            if (c == unwanted1 || c == unwanted2) colorMatch = false;

            if (colorMatch) {
                // Compute angles
                double xAngle = Math.toRadians(detection.getTargetYDegrees());
                double yAngle = Math.toRadians(detection.getTargetXDegrees());

                telemetry.addData("X T Angle", detection.getTargetYDegrees());
                telemetry.addData("Y T Angle", detection.getTargetXDegrees());
                telemetry.addData("X Angle", xAngle);
                telemetry.addData("Y Angle", yAngle);

                // Compute distances
                double xDistance = (((limelightHeight * 2) * Math.sin(xAngle)) / Math.sin(Math.toRadians(150)-xAngle));
                double yDistance = Math.tan(yAngle) * xDistance;

                telemetry.addData("X Distance", xDistance);
                telemetry.addData("Y Distance", yDistance);

                // Score based on alignment
                /*double rotationScore = -Math.abs((detection.getTargetCorners().get(0).get(0) -
                        detection.getTargetCorners().get(1).get(0)) /
                        (detection.getTargetCorners().get(1).get(1) -
                                detection.getTargetCorners().get(2).get(1)) - (1.5 / 3.5)); */
                double score = -yDistance + Math.abs(xDistance); // Weighted scoring

                double angle = 0;

                if (detection.getTargetCorners() == null || detection.getTargetCorners().size() < 4) {
                    angle = Double.NaN;
                }

//                List<List<Double>> corners = detection.getTargetCorners();

               // double dx = Math.toRadians(corners.get(1).get(0) - corners.get(0).get(0));
                //double dy = Math.toRadians(corners.get(2).get(1) - corners.get(0).get(1));
                angle = Double.NaN;

                scoredDetections.add(new LL3ADetection(detection, score, yDistance, xDistance, 0, angle));
            }
        }

        // Find the best detection

        if (!scoredDetections.isEmpty()) {
            scoredDetections.sort(Comparator.comparingDouble(LL3ADetection::getScore).reversed());
            LL3ADetection bestDetection = scoredDetections.get(0);

            bestAngle = bestDetection.getAngle();

            // Convert to coordinates and apply claw offsets
            sample = new Pose(
                    bestDetection.getXDistance(),
                    bestDetection.getYDistance(),
                    0
            );

            difference = new Pose(sample.getX() - intakeForwardOffset, sample.getY() + intakeLateralOffset, 0);

            target = new Pose(f.getPose().getX() + difference.getY(), f.getPose().getY(), f.getPose().getHeading());
            intake = (int)((Math.abs(sample.getX())/8)*600);


            cachedTarget = target.copy();

            toTarget = new PathBuilder()
                    .addPath(new BezierLine(f.getPose(), target)).setConstantHeadingInterpolation(f.getPose().getHeading()).build();

            telemetry.addData("Best Detection", bestDetection.getDetection().getClassName());
            telemetry.addData("Sample Position", "X: %.2f, Y: %.2f", sample.getX(), sample.getY());
            telemetry.addData("diff", difference);
            telemetry.addData("target", target);
            telemetry.addData("current", f.getPose());
        } else {
            target = cachedTarget.copy();
        }
    }

    public Pose getTarget() {
        return target;
    }

    public PathChain toTarget() {
        toTarget = new PathBuilder()
                .addPath(new BezierLine(f.getPose(), target))//new Pose(target, target.getY())))
                .setConstantHeadingInterpolation(f.getPose().getHeading())
                .build();
        return toTarget;
    }

    public void off() {
        limelight.stop();
    }

    public void on() {
        limelight.start();
    }
    public int positie(){return intake;}

    public double getAngle() {
        return bestAngle;
        //return manualInput.getPose().getHeading();
    }
}
