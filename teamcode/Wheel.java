package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Wheel {
    private DcMotor theMotor;
    private double radius;
    private int ticks;
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
        return distance/radius;
    }

    public int getNumOfTicks(){
        double dpr= getDPR();
        return (int) dpr*ticks; //Add in a rounding method
    }

    @Override
    public String toString(){
        return "Position: +"+position+"; Radius: "+radius+"; DPR: "+getDPR()+" ticks: "+ticks;
    }


}
