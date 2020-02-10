package org.firstinspires.ftc.teamcode.FIRE_team;
/**
 *In this section we import gyro,AngleUnit,AxesOrder and AxesReference
 */

import android.os.Process;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class DistanceToTargetFinder implements Runnable{
    /**
     *this class finds the distance to a target
     */
    private ActiveLocation activeLocation;
    private BNO055IMU imu;
    private volatile Location newPoint;
    private volatile double[] distanceTotarget;
    private volatile boolean stop ;

    /**
     * this function stops the thread
     * @param stop
     */
    public void setStop(boolean stop) {
        this.stop = stop;
    }

    /**
     * this function allows us  to use variables from other classes
     * @param activeLocation
     * @param imu
     */
    public DistanceToTargetFinder(ActiveLocation activeLocation, BNO055IMU imu)
    {
        this.activeLocation = activeLocation;
        this.imu = imu;
        distanceTotarget = new double[2];
        stop = false;
    }

    /**
     * this function calculates the distance between the robot and the target in each axes
     */

    public void DistanceAxisMeasurement( ){
        synchronized (this) {
            double GyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            double deltaX = newPoint.getX_axis() - activeLocation.getX_Axis();
            double deltaY = newPoint.getY_axis() - activeLocation.getY_Axis();


            distanceTotarget[1] = (-(deltaX * Math.sin(Math.toRadians(GyroAngle))) + deltaY * Math.cos(Math.toRadians(GyroAngle)));
            distanceTotarget[0] = deltaX * Math.cos(Math.toRadians(GyroAngle)) + deltaY * Math.sin(Math.toRadians(GyroAngle));
        }

    }


    @Override
    /**
     * this function activate the thread
     */
    public void run() {
        android.os.Process.setThreadPriority((Process.THREAD_PRIORITY_BACKGROUND));
        stop = false;
        while (!stop){
            DistanceAxisMeasurement();
        }
    }

    /**
     * this function allows the function DistanceAxisMeasurement to use the object newPoint in class Location
     * @param newPoint
     */
    public void setNewPoint(Location newPoint) {
        this.newPoint = newPoint;
        DistanceAxisMeasurement();
    }

    /**
     * this function allows use of the distanceTotarget outside of this class/thread
     * @return raw value of distanceTotarget in form of a double variable
     */
    public double[] getDistanceTotarget() {
        return distanceTotarget;
    }
}

