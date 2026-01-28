package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Common.CommonLogic;


/**
 * Base class for FTC Team 8492 defined hardware
 */
public class Intake extends BaseHardware{


    /**
     * The {@link #telemetry} field contains an object in which a user may accumulate data which
     * is to be transmitted to the driver station. This data is automatically transmitted to the
     * driver station on a regular, periodic basis.
     */
    public Telemetry telemetry = null;

    /**
     * Hardware Mappings
     */
    public HardwareMap hardwareMap = null; // will be set in Child class
    private TransitionRoller transitionRoller = new TransitionRoller();


    /**
     * BaseHardware constructor
     * <p>
     * The op mode name should be unique. It will be the name displayed on the driver station. If
     * multiple op modes have the same name, only one will be available.
     */
    private DcMotorEx NTKM01;
    private Servo PeaLight;
    public ColorRangeSensor NTKAP2;
    public ColorRangeSensor NTKAP3;
    //public LED PeaLight;  //off
    //public LED green_PeaLight; //on
    //public LED yellow_Pealight; //transitionroller of intake running

    public Mode CurrentMode;
    public Distance2 CurrentDistance2;
    public Distance3 CurrentDistance3;
    public Color CurrentColor;

    private double NTKM01Power;

    public final double minPower = -1.0;
    public final double maxPower = 1.0;

    public static final double stopSpeed = 0;
    public static final double inSpeed = -1;
    public static final double outSpeed = 0.65;
    public static final double autoSpeed = -1.0;


    public static final double Green = 0.5;
    public static final double Red = 0.28;
    public static final double Yellow = 0.388;
    public static final double Purple = 0.722;
    public static final double Blue = 0.6111;
    public static final double Orange = 0.333;
    public static final double Off = 0;

    private double NTKAP2distance;
    private double NTKAP3distance;


    public boolean AtIntakeStop = true;
    public boolean initLight1 = false;
    public boolean initLight2 = false;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime timerun = new ElapsedTime();
    private ElapsedTime sensorTime= new ElapsedTime();
    private ElapsedTime loopTime= new ElapsedTime();
    private ElapsedTime initLightTime = new ElapsedTime();

    private double targRange = 10.2; //in cm
    /**
     * User defined init method
     * <p>
     * This method will be called once when the INIT button is pressed.
     */
    public void init() {

        NTKAP3 = hardwareMap.get(ColorRangeSensor.class, "NTKAP3");
        NTKAP2 = hardwareMap.get(ColorRangeSensor.class, "NTKAP2");
        NTKM01 = hardwareMap.get(DcMotorEx.class, "NTKM01");
        PeaLight = hardwareMap.get(Servo.class,"PeaLight");
        //green_PeaLight = hardwareMap.get(LED.class,"green_PeaLight");
        //yellow_PeaLight = hardwareMap.get(LED.class,"yellow_PeaLight");

        //cmdRED();

        sensorTime.reset();
        initLightTime.reset();

        initLight1 = true;

    }

    /**
     * User defined init_loop method
     * <p>
     * This method will be called repeatedly when the INIT button is pressed.
     * This method is optional. By default this method takes no action.
     */
    public void init_loop(){

        if(initLight1 && initLightTime.milliseconds() >= 750){
            cmdORANGE();
            initLight1 = false;
            initLightTime.reset();
            initLight2 = true;
        }

        if(initLight2 && initLightTime.milliseconds() >= 750){
            cmdOFF();
            initLight2 = false;
            initLightTime.reset();
            initLight1 = true;
        }


    }

    /**
     * User defined start method.
     * <p>
     * This method will be called once when the PLAY button is first pressed.
     * This method is optional. By default this method takes not action.
     * Example usage: Starting another thread.
     */
    public void start(){
        initLight1 = false;
        initLight2 = false;
        cmdRED();
        loopTime.reset();

    }

    /**
     * User defined loop method
     * <p>
     * This method will be called repeatedly in a loop while this op mode is running
     */
    public void loop(){

        if(loopTime.milliseconds() >= 250) {
            if (CurrentMode == Mode.NTKforward) {
                if (CurrentDistance2 == Distance2.FILLED2 && CurrentDistance3 == Distance3.FILLED3 || ((CommonLogic.inRange(getMotorRPM(NTKM01), 600, 600)))) {
                    if (CurrentColor == Color.BLUE) {
                        cmdStop();
                    }
                }
            }

            if (CurrentMode == Mode.NTKforward) {
                sensorTime.reset();
            }

            //possibly put 400 time with sensortime for less glitching if glitching occurs.
            //if(sensorTime.milliseconds() >= 400) {
            getDistNTKAP2();
            getDistNTKAP3();
            //}

            if (NTKAP2distance <= 10 && sensorTime.milliseconds() >= 1000) {
                CurrentDistance2 = Distance2.FILLED2;
            } else {
                CurrentDistance2 = Distance2.MISSING2;
            }

            if (NTKAP3distance <= 10 && sensorTime.milliseconds() >= 1000) {
                CurrentDistance3 = Distance3.FILLED3;
            } else {
                CurrentDistance3 = Distance3.MISSING3;
            }

            loopTime.reset();

        }

        if(transitionRoller.CurrentMode == TransitionRoller.Mode.Spin &&
        CurrentMode == Mode.NTKforward){



        }

    }

    /**
     * User defined stop method
     * <p>
     * This method will be called when this op mode is first disabled
     * <p>
     * The stop method is optional. By default this method takes no action.
     */
    void stop(){

    }

    public void cmdBackward(){
        CurrentMode = Mode.NTKbackward;
        NTKM01.setPower (outSpeed);
        cmdGREEN();
        loopTime.reset();

    }
    public void cmdFoward(){
        CurrentMode = Mode.NTKforward;
        NTKM01.setPower (inSpeed);
        sensorTime.reset();
        loopTime.reset();
        cmdGREEN();

    }

    public void cmdStop(){
        CurrentMode = Mode.NTKstop;
        NTKM01.setPower (stopSpeed);
        cmdRED(); //line not needed when finished
        loopTime.reset();


    }

    public void cmdAutoFoward(){
        CurrentMode = Mode.NTKautoIn;
        NTKM01.setPower (autoSpeed);


    }

    public void cmdRED(){
        PeaLight.setPosition(Red);
        CurrentColor = Color.RED;

    }

    public void cmdGREEN(){
        PeaLight.setPosition(Green);
        CurrentColor = Color.GREEN;

    }

    public void cmdYELLOW(){
        PeaLight.setPosition(Yellow);
        CurrentColor = Color.YELLOW;

    }

    public void cmdPURPLE(){
        PeaLight.setPosition(Purple);
        CurrentColor = Color.PURPLE;

    }

    public void cmdBLUE(){
        PeaLight.setPosition(Blue);
        CurrentColor = Color.BLUE;
    }

    public void cmdORANGE(){
        PeaLight.setPosition(Orange);
        CurrentColor = Color.ORANGE;

    }

    public void cmdOFF(){
        PeaLight.setPosition(Off);
        CurrentColor = Color.OFF;

    }

    private void getDistNTKAP2() {
        NTKAP2distance = NTKAP2.getDistance(DistanceUnit.CM);
    }
    private void getDistNTKAP3() {
        NTKAP3distance = NTKAP3.getDistance(DistanceUnit.CM);
    }

    public enum Distance3 {
        FILLED3,
        MISSING3
    }

    public enum Distance2 {
        FILLED2,
        MISSING2
    }


    public enum Mode {
        NTKstop,
        NTKforward,
        NTKautoIn,
        NTKbackward
    }

    public enum Color {
        GREEN,
        RED,
        YELLOW,
        PURPLE,
        BLUE,
        ORANGE,
        OFF
    }

    public double getMotorRPM(DcMotorEx motor){
        double ticksPerRevolution = 28; //update and double check
        double gearRatio = 1.0; //update and double check
        double ticksPerSecond = motor.getVelocity();
        return (ticksPerSecond / ticksPerRevolution) * 60 * gearRatio;
    }








}
