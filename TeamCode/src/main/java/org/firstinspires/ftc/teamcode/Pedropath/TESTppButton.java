package org.firstinspires.ftc.teamcode.Pedropath;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.BaseHardware;



@TeleOp(name = "TESTppButton",  group = "Teleop")
public class TESTppButton extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        follower.setPose(new Pose(72, 72, 0));
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            if (gamepad1.a) {
                runPath();
            }

            Pose currentPose = follower.getPose();
            telemetry.addData("X", currentPose.getX());
            telemetry.addData("Y", currentPose.getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.getHeading()));
            telemetry.update();
        }
    }

    private void runPath() {
    }


    private void runExamplePath() {

        follower.followPath(
                follower.pathBuilder()
                        .setLinearHeadingInterpolation(32.5, 32.5) // Move forward
                        .setGlobalTangentHeadingInterpolation().build() // Turn right
        );


        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
            telemetry.addLine("Running path...");
            telemetry.update();
        }
    }
}




