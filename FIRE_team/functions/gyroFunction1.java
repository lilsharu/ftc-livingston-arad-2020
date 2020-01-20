package org.firstinspires.ftc.teamcode.functions;

import com.qualcomm.robotcore.util.Range;

public class gyroFunction1 implements gyroProportional{
    /**
     * spins the robot in auto with proportion
     * @param angleToReach
     * @param gyroAngle
     * @param slowAngle
     * @param Vmax
     * @return the turn speed
     */
    public double gyroProportionalCalculation(double angleToReach, double gyroAngle, double slowAngle, double Vmax){
        if (slowAngle == 0) return 0;
        Double turn =   ((angleToReach - gyroAngle)/slowAngle)*Vmax;
        Double turnAfterClip = Range.clip(turn ,-Vmax ,Vmax);
        return turnAfterClip;
    }
}
