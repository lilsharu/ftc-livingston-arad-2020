package org.firstinspires.ftc.teamcode;

import android.os.Process;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.ActiveLocation;
import org.firstinspires.ftc.teamcode.Location;

public class DistanceToTargetFinder implements Runnable{
    /**
     *
     */
    private ActiveLocation activeLocation;
    private BNO055IMU imu;
    private volatile Location newPoint;
    private volatile double[] distanceTotarget;
    private volatile boolean stop ;

    public void setStop(boolean stop) {
        this.stop = stop;
    }


    public DistanceToTargetFinder(ActiveLocation activeLocation, BNO055IMU imu)
    {
        this.activeLocation = activeLocation;
        this.imu = imu;
        distanceTotarget = new double[2];
        stop = false;
    }


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
    public void run() {
        android.os.Process.setThreadPriority((Process.THREAD_PRIORITY_BACKGROUND));
        stop = false;
        while (!stop){
            DistanceAxisMeasurement();
        }
    }

    public void setNewPoint(Location newPoint) {
        this.newPoint = newPoint;
        DistanceAxisMeasurement();
    }

    public double[] getDistanceTotarget() {
        return distanceTotarget;
    }
}

