package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.functions.driveFunction1;
import org.firstinspires.ftc.teamcode.functions.driveProportional;
import org.firstinspires.ftc.teamcode.functions.gyroFunction1;
import org.firstinspires.ftc.teamcode.functions.gyroProportional;

public class AutoDrivingSecondTry {

        private DcMotor leftSide;
        private DcMotor rightSide;
        private DcMotor middleMotor;
        private BNO055IMU imu;
    //    private Robot_Prameters robotPrameters;
    //    private Field field;
        private Telemetry telemetry;
   //     private Location currentRobotLocation ;
        private ElapsedTime runtime = new ElapsedTime();
        private ExampleForUsingLocaionControl EC ;
        private DistanceToTargetFinder distanceToTargetFinder;
        private ActiveLocation activeLocation;
        private driveProportional driveProportional;
        private gyroProportional gyroProportional;
        private Thread targetLocationThread;
        private Thread currentLocationThread;
        private LinearOpMode linearOpMode;
      //  double autoSpeed = 0.5;


        public AutoDrivingSecondTry (DcMotor leftSide, DcMotor rightSide, DcMotor middleMotor,
                           BNO055IMU imu/*, Field field */, Telemetry telemetry , LinearOpMode linearOpMode)
        {
            this.linearOpMode = linearOpMode  ;
            driveProportional = new driveFunction1() ;
            gyroProportional = new gyroFunction1();
            this.leftSide = leftSide;
            this.rightSide = rightSide;
            this.middleMotor = middleMotor;
            this.imu = imu;
            //this.robotPrameters = robotPrameters;
        //    this.field = field;
      //      currentRobotLocation = robotPrameters.getCenterOfTheRobot() ;
            this.telemetry = telemetry;
            activeLocation = new ActiveLocation(leftSide, rightSide, middleMotor, imu , new Location(0,0));
            currentLocationThread = new Thread(activeLocation);
            currentLocationThread.start();

            distanceToTargetFinder = new DistanceToTargetFinder(activeLocation, imu);
            targetLocationThread = new Thread(distanceToTargetFinder);
            targetLocationThread.start();
        }

        public void stopAllAutoCalculations(){
            distanceToTargetFinder.setStop(true);
            activeLocation.setStop(true);
        }

        public void setPosition(Location locationToReach , double angleToReach , double distanceRange , double slowDrive , double angleRange , double slowAngle , double Vmax){
            try {
                telemetry.addData("Begining is Active still alive: ", currentLocationThread.isAlive());
                telemetry.addData("Begining is finder still alive:", targetLocationThread.isAlive());
                telemetry.update();



//            ElapsedTime runtime = new ElapsedTime();
//            while (runtime.time() < 2000){}

                distanceToTargetFinder.setNewPoint(locationToReach);

                double[] distancedToDrive = distanceToTargetFinder.getDistanceTotarget();
                double gyroAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double rightMotorSpeed;
                double leftMotorSpeed;
                double middleMotorSpeed;
                while (linearOpMode.opModeIsActive() && !(Math.abs(distancedToDrive[0]) < distanceRange && Math.abs(distancedToDrive[1]) < distanceRange && Math.abs(angleToReach-gyroAngle) < angleRange)) {
                    distancedToDrive = distanceToTargetFinder.getDistanceTotarget();
                    telemetry.addData("encoders: left ",leftSide.getCurrentPosition());
                    telemetry.addData("encoders: right ",rightSide.getCurrentPosition());
                    telemetry.addData("encoders: midddle ",middleMotor.getCurrentPosition());
                    telemetry.addData("x axis distance: ", distancedToDrive[0]);
                    telemetry.addData("y axis distance: ", distancedToDrive[1]);
                    telemetry.addData("x current: ", activeLocation.getX_Axis());
                    telemetry.addData("y current: ", activeLocation.getY_Axis());



                    gyroAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                    double DrivePropValStraight = driveProportional.driveProportionalFunction(distancedToDrive[1], slowDrive , Vmax);
                    double DrivePropValMiddle = driveProportional.driveProportionalFunction(distancedToDrive[0], slowDrive , Vmax);
                    double gyroPropVal = gyroProportional.gyroProportionalCalculation(angleToReach, gyroAngle, slowAngle, Vmax/2);
                    double Max = Math.max(Math.abs(distancedToDrive[0]), Math.abs(distancedToDrive[1]));
                    double driveProp = distancedToDrive[1] * (-DrivePropValStraight);
                    double firstCalc = 1 - Math.abs(gyroPropVal);

                    Max = Math.max(Max ,1.0 );
                    rightMotorSpeed = ((firstCalc) * (driveProp) / Max) + gyroPropVal;
                    leftMotorSpeed = ((firstCalc) * (driveProp) / Max) - gyroPropVal;
                    middleMotorSpeed = distancedToDrive[0] * DrivePropValMiddle / Max;

                    telemetry.addData("left speed ", leftMotorSpeed);
                    telemetry.addData("middle speed: ", middleMotorSpeed);
                    telemetry.addData("right speed: ", rightMotorSpeed);
                    telemetry.addData("gyro angle: ", gyroAngle);
                    telemetry.addData("first driveprop caculation: ", driveProp);
                    telemetry.addData("first 1-gyroPropVal caculation: ", firstCalc);
                    telemetry.update();

                    leftSide.setPower(leftMotorSpeed);
                    rightSide.setPower(rightMotorSpeed);
                    middleMotor.setPower(middleMotorSpeed);
                 //   Thread.sleep(50);
                }
                distancedToDrive = distanceToTargetFinder.getDistanceTotarget();
//                telemetry.addData("encoders: left ",leftSide.getCurrentPosition());
//                telemetry.addData("encoders: right ",rightSide.getCurrentPosition());
//                telemetry.addData("encoders: midddle ",middleMotor.getCurrentPosition());

                telemetry.addData("x current: ", activeLocation.getX_Axis());
                telemetry.addData("y current: ", activeLocation.getY_Axis());
                telemetry.addData("After while exit :x axis distance: ", distancedToDrive[0]);
                telemetry.addData("After while exit :y axis distance: ", distancedToDrive[1]);
                telemetry.addData("is Active still alive: ", currentLocationThread.isAlive());
                telemetry.addData("is finder still alive:", targetLocationThread.isAlive());

                telemetry.update();
                leftSide.setPower(0);
                rightSide.setPower(0);
                middleMotor.setPower(0);
//                runtime = new ElapsedTime();
//                while (runtime.time() < 10000) {
//                }
            }catch (Exception e){ telemetry.addData("error:",e.getStackTrace().toString());}

        }
}
