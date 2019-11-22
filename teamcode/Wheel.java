package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Wheel {
    private DcMotor theMotor;
    private double radius;
    private double ticks;
    private String position;

    public Wheel(DcMotor m, double r, int ticks, String pos){
        this.theMotor = m;
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
