package org.firstinspires.ftc.teamcode.Hardware;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.function.Supplier;

public class Robot extends BaseHardware {

    private static final Logger log = LoggerFactory.getLogger(Robot.class);
    public DriveTrain driveTrain = new DriveTrain();
    public Intake intake = new Intake();
    public Launcher launcher = new Launcher();
    public Uppies uppies = new Uppies();
    public TransitionRoller transitionRoller = new TransitionRoller();
    public LauncherBlocker launcherBlocker = new LauncherBlocker();
    public Limey limey = new Limey();


    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    public boolean bCkSenors = false;

    //auto align constants

    public double minTargetVertPos = 65; //63-69
    public double minTargetDist = 26;
    public double maxTargetVertPos = 169;
    public double maxTargetDist = 78;


    public double nominalTagWidthRatio = 0.95;
    public double nominalTagAngle = 0;
    public double extremeTagWidthRatio = 0.6829;
    public double extremeTagAngle = 75;
    public double tagExtremeRightPos = 296;
    public double tagExtremeRightAngle = 65;
    public double targetPointFromTag = 12;


    @Override
    public void init() {
        // Must set Hardware Map and telemetry before calling init
        driveTrain.hardwareMap = this.hardwareMap;
        driveTrain.telemetry = this.telemetry;
        driveTrain.init();


        //  lighting.hardwareMap = this.hardwareMap;
        //lighting.telemetry = this.telemetry;
        // lighting.init();

        // sensors.hardwareMap = this.hardwareMap;
        // sensors.telemetry = this.telemetry;
        // sensors.init();

        intake.hardwareMap = this.hardwareMap;
        intake.telemetry = this.telemetry;
        intake.init();

        launcher.hardwareMap = this.hardwareMap;
        launcher.telemetry = this.telemetry;
        launcher.init();

        launcherBlocker.hardwareMap = this.hardwareMap;
        launcherBlocker.telemetry = this.telemetry;
        launcherBlocker.init();

        transitionRoller.hardwareMap = this.hardwareMap;
        transitionRoller.telemetry = this.telemetry;
        transitionRoller.init();

        limey.hardwareMap = this.hardwareMap;
        limey.telemetry = this.telemetry;
        limey.setTelemetry(telemetry);
        limey.init();

        uppies.hardwareMap = this.hardwareMap;
        uppies.telemetry = this.telemetry;
        uppies.init();

    }

    @Override
    public void init_loop() {
        driveTrain.init_loop();
        //lighting.init_loop();
        // sensors.init_loop();
        intake.init_loop();
        launcher.init_loop();
        launcherBlocker.init_loop();
        transitionRoller.init_loop();
        limey.init_loop();
        uppies.init_loop();
    }

    @Override
    public void start() {
        driveTrain.start();
        // lighting.start();
        // sensors.start();
        intake.start();
        launcher.start();
        launcherBlocker.start();
        transitionRoller.start();
        limey.start();
        uppies.start();


        // lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    @Override
    public void loop() {
        driveTrain.loop();
        //. lighting.loop();
        // sensors.loop();
        intake.loop();
        launcher.loop();
        launcherBlocker.loop();
        transitionRoller.loop();
        limey.loop();
        uppies.loop();

        if (transitionRoller.CurrentMode == TransitionRoller.Mode.Stop
                && intake.CurrentMode == Intake.Mode.NTKforward) {
            intake.cmdBLUE();
        }


    }

    public void autonLoop() {
        //driveTrain.loop();
        //. lighting.loop();
        // sensors.loop();
        intake.loop();
        launcher.loop();
        launcherBlocker.loop();
        transitionRoller.loop();
        limey.loop();
        uppies.loop();


    }


    @Override
    public void stop() {
        driveTrain.stop();
        // lighting.stop();
        // sensors.stop();
        intake.stop();
        launcher.stop();
        launcherBlocker.stop();
        transitionRoller.stop();
        limey.stop();
        uppies.stop();
        // lighting.UpdateBaseColor(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void safteyCheck() {
        //when called comfirm flicker is in safe position before spindexing.
    }


    public double targetDistanceCalc() {

        double targetOffsetAngle_Vertical = limey.getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 14.5;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 14.0;

        // distance from the target to the floor
        double goalHeightInches = 29.5;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        double distanceFromRobotToGoalInches = distanceFromLimelightToGoalInches + 0;  //shouldn't this be not zero?
        return distanceFromRobotToGoalInches;


    }

    public double targetAngleCalc() {

        if (launcher.CurrentPosition == Launcher.Position.LaunchFar) {


            double currentTagId = limey.getTagID();
            if (currentTagId != -1) {
                double targetOffsetAngle_Horizontal = limey.getTx();


                double tagAngle = limey.getTagAngle() + 90;
                double targetDistanceCalc = targetDistanceCalc();
                double hypotenuse = Math.sqrt((targetDistanceCalc * targetDistanceCalc) + (targetPointFromTag * targetPointFromTag) - (2 * (targetDistanceCalc * targetPointFromTag * Math.cos(Math.toRadians(tagAngle)))));

                double compensationAngle = 180 - tagAngle - Math.toDegrees(Math.asin(Math.sin(Math.toRadians(tagAngle)) * targetDistanceCalc) / hypotenuse);


                // if (driveTrain.getCurrentHeading() >= 90) {
                //   return defaultAngle;
                //} else if (driveTrain.getCurrentHeading() <= -90) {
                //  return -defaultAngle;
                if (currentTagId == 24) {
                    //compensate left
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal; //- compensationAngle;
                } else if (currentTagId == 20) {
                    //compensate right
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal - 5; //+ compensationAngle;
                } else {
                    return driveTrain.getCurrentHeading();
                }
            } else {
                double defaultAngle = 25;
                if (driveTrain.getCurrentHeading() >= 90) {
                    return defaultAngle;
                } else if (driveTrain.getCurrentHeading() <= -90) {
                    return -defaultAngle;

                }
            }
            //return driveTrain.getCurrentHeading();

        } else {
            double currentTagId = limey.getTagID();
            if (currentTagId != -1) {
                double targetOffsetAngle_Horizontal = limey.getTx();


                double tagAngle = limey.getTagAngle() + 90;
                double targetDistanceCalc = targetDistanceCalc();
                double hypotenuse = Math.sqrt((targetDistanceCalc * targetDistanceCalc) + (targetPointFromTag * targetPointFromTag) - (2 * (targetDistanceCalc * targetPointFromTag * Math.cos(Math.toRadians(tagAngle)))));

                double compensationAngle = 180 - tagAngle - Math.toDegrees(Math.asin(Math.sin(Math.toRadians(tagAngle)) * targetDistanceCalc) / hypotenuse);


                // if (driveTrain.getCurrentHeading() >= 90) {
                //   return defaultAngle;
                //} else if (driveTrain.getCurrentHeading() <= -90) {
                //  return -defaultAngle;
                if (currentTagId == 24) {
                    //compensate left
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal -4; //- compensationAngle;
                } else if (currentTagId == 20) {
                    //compensate right
                    return driveTrain.getCurrentHeading() + targetOffsetAngle_Horizontal ; //+ compensationAngle;
                } else {
                    return driveTrain.getCurrentHeading();
                }
            } else {
                double defaultAngle = 25;
                if (driveTrain.getCurrentHeading() >= 90) {
                    return defaultAngle;
                } else if (driveTrain.getCurrentHeading() <= -90) {
                    return -defaultAngle;

                }
            }
            //return driveTrain.getCurrentHeading();

        }

        return driveTrain.getCurrentHeading();
    }
    /*
    public void findAlliance{
        double currentTagId = limey.getTagID();
        if(currentTagId == 24){

        }else if(currentTagId == 20){

        }else{

        }
    }
     */

}

















