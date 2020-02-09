package org.firstinspires.ftc.teamcode.FIRE_team;
/**
 * we import process , imu , DcMotor ,angle unit ,axes order  and axes reference
 */

import android.os.Process;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class ActiveLocation implements Runnable {
    /** Declare  the parts and verbals   */

    private volatile double X_Axis;
    private volatile double Y_Axis;
    private volatile boolean stop;
    private DcMotor motorLeft;
    private DcMotor motorRight;
    private DcMotor middleMotor;
    private BNO055IMU imu;
 //   private Location currentLocation;
    private volatile double deltaX;
    private volatile double deltaY;
    private volatile double deltaY1;
    private volatile double deltaY2;
    private volatile double current_Y1;
    private volatile double current_Y2;
    private volatile double current_X;
    private volatile double previous_Y1;
    private volatile double previous_Y2;
    private volatile double previous_X;
    private double fieldSide = 3660;

    /**
     * the contractor
     * @param motorLeft
     * @param motorRight
     * @param middleMotor
     * @param imu
     * @param currentLocation
     */
    public ActiveLocation(DcMotor motorLeft, DcMotor motorRight,  DcMotor middleMotor, BNO055IMU imu , Location currentLocation ) {
        this.motorLeft = motorLeft;
        this.motorRight = motorRight;
        this.middleMotor = middleMotor;
        this.imu = imu;

      //  this.currentLocation = currentLocation;
        stop = false;
        Y_Axis = currentLocation.getY_axis();
        X_Axis = currentLocation.getX_axis();
    }

    @Override
    public void run() {
        /** and
         * set up the verbals ThreadPriority
         */
        android.os.Process.setThreadPriority((Process.THREAD_PRIORITY_BACKGROUND));
        current_X = 0;
        current_Y1 = 0;
        current_Y2 = 0;
        previous_X = 0;
        previous_Y1 = 0;
        previous_Y2 = 0;
        deltaX = 0;
        deltaY = 0;
        deltaY1 = 0;
        deltaY2 = 0;
        stop = false;
        while (!stop){
            oppositeCalculation();
        }
    }

    /**
     * the opposite Calculation to improve accuracy of the driving
     */
    public void oppositeCalculation()
    {

        previous_Y1 = current_Y1;
        previous_Y2 = current_Y2;
        previous_X = current_X;

        double angle;
        synchronized (this) {
            current_X = AutoDriving.convertToDistanceX(middleMotor.getCurrentPosition());
            current_Y1 = AutoDriving.convertToDistanceY(motorLeft.getCurrentPosition());
            current_Y2 = AutoDriving.convertToDistanceY(motorRight.getCurrentPosition());
            angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        }
        deltaX = current_X - previous_X;
        deltaY1 = current_Y1 - previous_Y1;
        deltaY2 = current_Y2 - previous_Y2;
        deltaY = (deltaY1 + deltaY2) / 2;


        X_Axis += deltaX * Math.cos(Math.toRadians(angle)) - deltaY * Math.sin(Math.toRadians(angle));
        Y_Axis += deltaX * Math.sin(Math.toRadians(angle)) + deltaY * Math.cos(Math.toRadians(angle));

    }

    /**
     * gets the x axis
     * @returnthe x axis
     */
    public double getX_Axis() {
        return X_Axis;
    }

    /**
     * gets the y axis
     * @returnthe y axis
     */
    public double getY_Axis() {
        return Y_Axis;
    }

    /**
     * checks if the thead stopped
     * @return stop
     */
    public boolean isStop() {
        return stop;
    }

    /**
     * stops the thread
     * @param stop
     */
    public void setStop(boolean stop) {
        this.stop = stop;
    }

    /**
     * updates the location after moving the foundation
     * @param frontDistance
     * @param sideDistance
     */
    public void updateCurrentLocationWithSensors(double frontDistance, double sideDistance)
    {
        Y_Axis = sideDistance;
        X_Axis = frontDistance;
    }
}
