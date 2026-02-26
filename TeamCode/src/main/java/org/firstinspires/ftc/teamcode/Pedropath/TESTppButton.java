package org.firstinspires.ftc.teamcode.Pedropath;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import org.firstinspires.ftc.teamcode.PedroPathing; // Adjust import to your PedroPathing package


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.PedroPathing.CompBotConstants;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

import pedropathing.path.Path;
import pedropathing.path.PathBuilder;
import pedropathing.localization.Pose;

@TeleOp(name = "TeleOp with PedroPathing Parking", group = "TeleOp")
public class TeleOpWithPedroParking extends LinearOpMode {

    private PedroPathing drive;
    private boolean parkingMode = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize PedroPathing drive system
        drive = new PedroPathing(hardwareMap);

        // Set starting pose (middle of the field)
        drive.setPoseEstimate(new Pose(0, 0, Math.toRadians(0)));

        // Build path from middle to parking zone
        Path toParking = new PathBuilder()
                .addBezierCurve(new Pose(0, 0), new Pose(24, -48)) // Example coordinates
                .build();

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Manual driving when not in parking mode
            if (!parkingMode) {
                double drivePower = -gamepad1.left_stick_y;
                double strafePower = gamepad1.left_stick_x;
                double turnPower = gamepad1.right_stick_x;
                drive.setWeightedDrivePower(drivePower, strafePower, turnPower);

                // Trigger parking mode with button press
                if (gamepad1.a) {
                    parkingMode = true;
                    drive.followPathAsync(toParking);
                }
            } else {
                // Let PedroPathing handle movement
                drive.update();

                // Stop parking mode when path is done
                if (!drive.isBusy()) {
                    parkingMode = false;
                }
            }

            telemetry.addData("Mode", parkingMode ? "Parking" : "Manual");
            telemetry.update();
        }
    }
}









