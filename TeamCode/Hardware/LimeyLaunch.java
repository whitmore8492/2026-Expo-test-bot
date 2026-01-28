package org.firstinspires.ftc.teamcode.Hardware;

public class LimeyLaunch {

    private Limey limey;
    private Launcher launcher;


    public LimeyLaunch(Limey limey,Launcher launcher){
        this.limey = limey;
        this.launcher = launcher;

    }
    public void update(){
        double tx = limey.getTx();
        double ty = limey.getTy();
        double yaw = limey.getTagAngle();


        double[] rpms = calculateRPMs(tx, ty, yaw);

        //launcher.setTargetRPMs(rpms[0],rpms[1]);
    }
    private double[] calculateRPMs(double tx, double ty, double yaw){

        double baseTop = 5970;
        double baseBottom = 5980;

        double distanceAdjust = ty * -25;
        double angleAdjust = Math.abs(tx) * 10;
        double yawAdjust = Math.abs(yaw) * 5;

        double top = baseTop + distanceAdjust + angleAdjust + yawAdjust;
        double bottom = baseBottom + distanceAdjust + angleAdjust + yawAdjust;

        return new double[]{top, bottom};
    }


}
