package org.firstinspires.ftc.teamcode.FIRE_team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AutoDriving {
    /**
     *      Declare parts from the Hardware class
     *      and auto speed
     */
    private DcMotor leftSide;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    private BNO055IMU imu;
    private Robot_Prameters robotPrameters;
    private Field field;
    private Telemetry telemetry;
    private Location currentRobotLocation ;
    private ElapsedTime runtime = new ElapsedTime();
    private ExampleForUsingLocaionControl EC ;


    double autoSpeed = 0.5;

    /**
     * init the parts
     * @param leftSide
     * @param rightSide
     * @param middleMotor
     * @param imu
     * @param robotPrameters
     * @param field
     * @param telemetry
     */

    public AutoDriving(DcMotor leftSide, DcMotor rightSide, DcMotor middleMotor,
                       BNO055IMU imu, Robot_Prameters robotPrameters, Field field , Telemetry telemetry)
    {

        //,ExampleForUsingLocaionControl EC
        //this.EC = EC ;
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        this.middleMotor = middleMotor;
        this.imu = imu;
        this.robotPrameters = robotPrameters;
        this.field = field;
        currentRobotLocation = robotPrameters.getCenterOfTheRobot() ;
        this.telemetry = telemetry;

    }

    /**
     * rotate the robot to received angle
     * @param angle
     * @return true if the function run done. false if there are some errors
     */
    public void RotateRobotToAngle (double angle,double range ,double slow ,double Vmax){

        leftSide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Rotate( angle, range , slow , Vmax);

        leftSide.setPower(0);
        rightSide.setPower(0);
    }

    /**
     * rotate the robot
     * @param angle
     * @param range
     * @param slow
     * @param Vmax
     */
    private void Rotate(double angle,double range ,double slow ,double Vmax){
        while ( ! (Math.abs(angle-GyroAngle()) <= range) ){
            Double turn =   ((angle-GyroAngle())/slow)*Vmax;
            Double turnAfterClip = Range.clip(turn ,-Vmax ,Vmax);
            leftSide.setPower(-turnAfterClip);
            rightSide.setPower(turnAfterClip);

            telemetry.addData("", "current angle: " + GyroAngle());
            telemetry.addData("Motors", "Target angle: " + angle);
            telemetry.update();
        }
        leftSide.setPower(0);
        rightSide.setPower(0);
    }

    /**
     * gets the gyro angel
     * @return gyro angel
     */
    private double GyroAngle(){
        return  -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle ;
    }

    /**
     * drive the robot to Landmark
     * @param newPoint
     * @return true if the function run done. false if there are some errors
     */
    public boolean DriveToCordinate(Location newPoint , double range, double angle) {
        // use currentLocation and update it at the end , and use current gyro angle
        // use function in Location class Location.DistanceAxisMeasurement(......)
        // code here
        double[] distances = newPoint.DistanceAxisMeasurement(currentRobotLocation, newPoint, angle);


//        telemetry.addData("x, y", distances[0] + " , " + distances[1]);
//        telemetry.update();
//        runtime.reset();
//        while (runtime.milliseconds() < 2000);

/**
 * set the mod of the motors
 */
        rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean a = true;
        boolean b = true;
        int counter = 0;
        double[] yMotorValues = null;

        while (a || b) {
            if(Math.abs(middleMotor.getCurrentPosition() - convertToTicksX(distances[0])) > range)
            {
               if(middleMotor.getCurrentPosition() - convertToTicksX(distances[0]) < 0) {
                   middleMotor.setPower(-autoSpeed);
               }
               else
               {
                   middleMotor.setPower(autoSpeed);
               }
            }
            else
            {
                middleMotor.setPower(0);
                a = false;
            }

            if(Math.abs(leftSide.getCurrentPosition() - convertToTicksY(distances[1])) > range)
            {
//                yMotorValues = drivingFixWithGyro(-proprtionalDrive(convertToDistanceY(leftSide.getCurrentPosition())
//                        ,1000, distances[1]), angle);
//
//                leftSide.setPower(yMotorValues[0]);
//                rightSide.setPower(yMotorValues[1]);



                telemetry.addData("left encoder: ", convertToDistanceY(leftSide.getCurrentPosition()));
                telemetry.addData("middle encoder: ", convertToDistanceY(middleMotor.getCurrentPosition()));
                telemetry.addData("angle: ", "current angle: " + GyroAngle());
                telemetry.addData("Motors", "Target angle: " + angle);
                telemetry.update();

                yMotorValues = drivingFixWithGyro(-autoSpeed, angle);

                if(leftSide.getCurrentPosition() - convertToTicksY(distances[1]) < 0) {


                    leftSide.setPower(-autoSpeed);
                    rightSide.setPower(-autoSpeed);
                }
                else
                {

                    leftSide.setPower(autoSpeed);
                    rightSide.setPower(autoSpeed);
                }
            }
            else
            {
                rightSide.setPower(0);
                leftSide.setPower(0);

                b = false;
            }

            if(counter == 50)
            {
                currentRobotLocation = currentRobotLocation.oppositeCalculation(convertToDistanceX(middleMotor.getCurrentPosition()),
                        convertToTicksY(leftSide.getCurrentPosition()), GyroAngle());
                counter = 0;
                telemetry.addData("current cordinates: (%.2f) (%.2f)", currentRobotLocation.getX_axis() + "," + currentRobotLocation.getY_axis());
                telemetry.update();

            }
            else
            {
                counter++;
            }
        }


         leftSide.setPower(0);
         middleMotor.setPower(0);
         rightSide.setPower(0);
         return true;
    }



    private double[] drivingFixWithGyro(double motor_speed , double currntAngle){

        double value_left_motor = motor_speed;
        double value_right_motor = motor_speed;

        double incline = 30/8; //  15/8


       if(Math.abs(GyroAngle() - currntAngle) > 1 && Math.abs(GyroAngle() - currntAngle) < 10)
        {
//            value_left_motor = value_left_motor*(-1*((incline * (GyroAngle()-currntAngle) + 1.25)/100) + 1);
//            value_right_motor =value_left_motor;
//         //  value_right_motor = value_right_motor*(((incline * (GyroAngle()-currntAngle) + 1.25)/100) + 1);
//            double max = Math.max( Math.abs(value_left_motor) , Math.abs(value_right_motor));
//            if(max > 1) {
//                value_left_motor = value_left_motor / max;
//                value_right_motor = value_left_motor;
////                value_right_motor = value_right_motor / max;
//            }
            if(GyroAngle() - currntAngle > -10){
//                if(value_right_motor > 0 ) {
//                    value_right_motor = -0.01;
//                }else{
//                    value_right_motor = 0.01;
//                }
                value_right_motor = value_right_motor / 10 ;
            }else if(GyroAngle() - currntAngle < 10){
//                if(value_left_motor > 0 ) {
//                    value_left_motor = -0.01;
//                }else{
//                    value_left_motor = 0.01;
//                }
                value_left_motor = value_left_motor / 10 ;
            }
        }
        else if(Math.abs(GyroAngle() - currntAngle) > 10)
        {
            middleMotor.setPower(0);
            runtime.reset();
            while(runtime.milliseconds() < 1000) ;
            Rotate(currntAngle, 20, Math.abs(currntAngle - GyroAngle()), 0.5);
        }

        double[] res = {value_left_motor , value_right_motor} ; // you must fix values
        return res ;
    }


    private double proprtionalDrive(double passedDistance, double slow, double targetDistance)
    {
        double incline = 0.7 / slow;
        double motorValue = 0;

        motorValue = incline * (targetDistance - passedDistance) + 0.3;

        return Range.clip(motorValue,-1,1);
    }


    public static double convertToDistanceX(double ticks){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return (ticks * WHEEL_DIAMETER_MM * Math.PI) / (DRIVE_GEAR_REDUCTION * 28.5);
    }


    public static double convertToDistanceY(double ticks) {
        double DRIVE_GEAR_REDUCTION = 20;
        double WHEEL_DIAMETER_MM = 90;
        return -((ticks * WHEEL_DIAMETER_MM * Math.PI) / (DRIVE_GEAR_REDUCTION * 28.5));
    }


    public static double convertToTicksX(double distance){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return (distance * DRIVE_GEAR_REDUCTION  * 28.5) / (WHEEL_DIAMETER_MM * Math.PI );
    }


    private double convertToTicksY(double distance){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return  -((distance * DRIVE_GEAR_REDUCTION  * 28.5)/ (WHEEL_DIAMETER_MM * Math.PI));

    }

}
