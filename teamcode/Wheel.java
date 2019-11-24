package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Wheel {
    private DcMotor theMotor;
    private String OrgUn; //Unit entered
    private String vertUn; //Unit to convert to
    private double radius;
    private double ticks; //Ticks per a rotation
    private String position; //String describing

    public Wheel(DcMotor m, double r, int ticks, String pos, String unit){
        this.theMotor = m;
        this.OrgUn = unit;
        this.radius = r;
        this.ticks = ticks;
        this.position = pos;
    }

    public Wheel(DcMotor m){
        this.theMotor = m;
    }



    public double getDPR(){

        return radius*2*Math.PI;
    }

    public double getNumOfRot(double distance){
        double dpr = getDPR();
        return distance/dpr;
    }

    public int getNumOfTicks(double dist){
        double nor= getNumOfRot(dist);
        return (int) Math.round(nor*ticks);
    }

    @Override
    public String toString(){
        return "Position: +"+position+"; Radius: "+radius+"; DPR: "+getDPR()+" ticks: "+ticks;
    }


}
