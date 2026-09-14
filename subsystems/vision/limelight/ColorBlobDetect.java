package org.firstinspires.ftc.teamcode.subsystems.vision.limelight;

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

public class ColorBlobDetect {

    // --- Fields (must be declared here, not just used in the constructor) ---
    private final Limelight3A limelight;
    private final IMUSubsystem imuSubsystem;
    private final Telemetry telemetry;
    private LLResult latestResult;

    public double distance;
    public double heading;

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
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        }

        distance = trigDistance.calculateDistance(
                LimelightConstants.cameraHeight - LimelightConstants.pollenTargetHeight,
                LimelightConstants.cameraAngle + latestResult.getTy()
        );

        heading = imuSubsystem.getHeading();
    }
}