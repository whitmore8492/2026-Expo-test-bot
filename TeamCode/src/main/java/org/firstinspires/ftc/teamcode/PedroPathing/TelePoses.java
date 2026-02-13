package org.firstinspires.ftc.teamcode.PedroPathing;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Common.Settings;
import org.firstinspires.ftc.teamcode.Hardware.Robot;


public class TelePoses {

    //Robot robot = new Robot();

    public static Follower follower;
    public static double powerNormal = 0.65;

    public Pose ypose = new Pose(follower.getPose().getX(), follower.getPose().getY(), follower.getHeading()); //Math.toRadians());
    public static Pose xpose = new Pose(72, 72, Math.toRadians(0));


    public static PathChain yxpath;

    public void runPaths(){

           yxpath =follower.pathBuilder()
            .addPath(new BezierLine(ypose, xpose))
            .setLinearHeadingInterpolation(ypose.getHeading(),xpose.getHeading())
            .build();

}


}
