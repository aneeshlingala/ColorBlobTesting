package org.firstinspires.ftc.teamcode.subsystems.vision.limelight;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

import org.firstinspires.ftc.teamcode.constants.LimelightConstants;
import org.firstinspires.ftc.teamcode.subsystems.hardware.imu.IMUSubsystem;
import org.firstinspires.ftc.teamcode.util.trigDistance;

import com.pedropathing.localization.PoseTracker;
import com.pedropathing.follower.Follower;

import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.Point;

public class ColorBlobDetect {

  
    private final Limelight3A limelight;
    private final IMUSubsystem imuSubsystem;
    private final Telemetry telemetry;
    private LLResult latestResult;

    public double distance;
    public double heading;

    public Pose targetBlobPose;

    public ColorBlobDetect(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.imuSubsystem = new IMUSubsystem(hardwareMap);
        this.limelight = hardwareMap.get(Limelight3A.class, LimelightConstants.LimelightName);
    }

    public void setupLimelight() {
        limelight.pipelineSwitch(LimelightConstants.ColorBlobPipeline);
        limelight.start();
    }

    public String getLog() {
        LLStatus status = limelight.getStatus();
        return "Name: " + status.getName()
                + ", Temp: " + status.getTemp() + "C"
                + ", CPU: " + status.getCpu() + "%"
                + ", FPS: " + (int) status.getFps()
                + ", Pipeline Index: " + status.getPipelineIndex()
                + ", Pipeline Type: " + status.getPipelineType();
    }

    public void update() {
        latestResult = limelight.getLatestResult();
    }

    public boolean isColorTargetValid() {
        return latestResult != null
                && latestResult.isValid()
                && !latestResult.getColorResults().isEmpty();
    }

    public void returnPoseFromClosestColor() {
        if (!isColorTargetValid()) return;

        List<LLResultTypes.ColorResult> colorResults = latestResult.getColorResults();

        LLResultTypes.ColorResult biggest = null;
        double biggestArea = -1;

        for (LLResultTypes.ColorResult cr : colorResults) {
            double area = cr.getTargetArea();

            telemetry.addData("Color", "X: %.2f, Y: %.2f, Area: %.2f",
                    cr.getTargetXDegrees(), cr.getTargetYDegrees(), area);

            if (area > biggestArea) {
                biggestArea = area;
                biggest = cr;
            }
        }

        if (biggest == null) return;

        distance = trigDistance.calculateDistance(
                LimelightConstants.cameraHeight - LimelightConstants.pollenTargetHeight,
                LimelightConstants.cameraAngle + biggest.getTargetYDegrees()
        );

        heading = imuSubsystem.getHeading();

        double targetAngle = follower.getPose().getHeading() + Math.toRadians(biggest.getTargetXDegrees());
        targetBlobPose = new Pose(
                follower.getPose().getX() + distance * Math.cos(targetAngle),
                follower.getPose().getY() + distance * Math.sin(targetAngle),
                targetAngle

        );

        PathChain path = follower.pathBuilder()
                .addPath(new BezierLine(follower.getPose(), targetBlobPose))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetBlobPose.getHeading())
                .build();

        follower.followPath(path);

        follower.followPath(path);
    }
}
