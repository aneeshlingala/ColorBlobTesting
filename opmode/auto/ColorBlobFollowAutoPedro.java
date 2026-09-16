package org.firstinspires.ftc.teamcode.opmode.auto;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.subsystems.vision.limelight.ColorBlobDetect;

@Autonomous(name = "Color blob follower | Pedro adaptive pathing", group = "Autonomous")
public class ColorBlobFollowAutoPedro extends OpMode {

    private ColorBlobDetect blobDetect;


    @Override
    public void init() {
        blobDetect = new ColorBlobDetect(hardwareMap, telemetry);
        blobDetect.setupLimelight();
    }



    @Override
    public void loop() {
        blobDetect.getLog();
        telemetry.addData("Log", blobDetect.getLog());
        telemetry.addData("Proper Target", blobDetect.isColorTargetValid());
        telemetry.addData("Following", follower.isBusy());
        follower.update();

        if (!follower.isBusy() && blobDetect.isColorTargetValid()) {
            Pose targetBlobPose = blobDetect.returnPoseFromClosestColor();

            if (targetBlobPose != null) {
                PathChain path = follower.pathBuilder()
                        .addPath(new BezierLine(follower.getPose(), targetBlobPose))
                        .setLinearHeadingInterpolation(follower.getPose().getHeading(), targetBlobPose.getHeading())
                        .build();

                follower.followPath(path);

            }
        }

    }

    @Override
    public void stop() {
        follower.pausePathFollowing();
    }
}
