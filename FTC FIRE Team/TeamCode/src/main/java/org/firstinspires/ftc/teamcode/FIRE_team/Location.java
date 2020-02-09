package org.firstinspires.ftc.teamcode.FIRE_team;
import java.lang.Math;


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


        distanceFor_Y_Axis = (-(deltaX * Math.sin(Math.toRadians(-GyroAngle))) + deltaY * Math.cos(Math.toRadians(-GyroAngle)));
        distanceFor_X_Axis = deltaX * Math.cos(Math.toRadians(-GyroAngle)) + deltaY * Math.sin(Math.toRadians(-GyroAngle));


        double[] res = {distanceFor_X_Axis , distanceFor_Y_Axis } ;
        return res ;
    }

    /**
     * calculate the current location from the passed  distance and the angle
     * @param passedDistanceX
     * @param passedDistanceY
     * @param gyroAngle
     * @return the current point of the robot
     */

    public Location oppositeCalculation(double passedDistanceX, double passedDistanceY, double gyroAngle)
    {

        double cordinateX = passedDistanceX * Math.cos(Math.toRadians(gyroAngle)) - passedDistanceY * Math.sin(Math.toRadians(gyroAngle));
        double cordinateY = passedDistanceX * Math.sin(Math.toRadians(gyroAngle)) + passedDistanceY * Math.cos(Math.toRadians(gyroAngle));

        Location CurrentPoint = new Location(cordinateX, cordinateY);

        return CurrentPoint;
    }


    /**
     * gets x axis
     * @return x axis
     */



    public double getX_axis() {
        return X_axis;
    }

    /**
     * gets y axis
     * @return y axis
     */

    public double getY_axis() {
        return Y_axis;
    }
}
