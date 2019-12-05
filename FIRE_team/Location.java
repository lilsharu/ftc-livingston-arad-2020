package org.firstinspires.ftc.teamcode;
import java.lang.Math;

import java.lang.reflect.Array;
import java.util.Map;

/**
 * Created by lior on 10/8/2019.
 */

public class Location {

    private double X_axis;
    private double Y_axis;

    /**
     * constructor for initialization
     * @param x_axis
     * @param y_axis
     */
    public Location(double x_axis, double y_axis) {
        X_axis = x_axis;
        Y_axis = y_axis;
    }

    /**
     * We will support only transference without changing robot Current angle
     * @param CurrentPoint
     * @param NewPoint
     * @param GyroAngle
     * @return distances for X and Y robot Axes
     */
    public static double[] DistanceAxisMeasurement(Location CurrentPoint , Location NewPoint , double GyroAngle){
        double distanceFor_X_Axis = 0;
        double distanceFor_Y_Axis = 0;
        double deltaX = NewPoint.getX_axis() - CurrentPoint.getX_axis();
        double deltaY = NewPoint.getY_axis() - CurrentPoint.getY_axis();


        distanceFor_X_Axis = (deltaX * Math.cos(Math.toRadians(-GyroAngle)) + deltaY * Math.sin(Math.toRadians(-GyroAngle)));
        distanceFor_Y_Axis = (-(deltaX * Math.sin(Math.toRadians(-GyroAngle))) + deltaY * Math.cos(Math.toRadians(-GyroAngle)));



        double[] res = {distanceFor_X_Axis , distanceFor_Y_Axis } ;
        return res ;
    }


    public double getX_axis() {
        return X_axis;
    }

    public double getY_axis() {
        return Y_axis;
    }
}
