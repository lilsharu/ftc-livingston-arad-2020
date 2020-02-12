package org.firstinspires.ftc.teamcode.FIRE_team;

/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

//import java.util.Observable;
//import java.util.Observer;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="LocationControlTest", group="Pushbot")

public class LocationControlTest extends LinearOpMode  {

    /* Declare OpMode members. */
    //Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    //private Servo fundationHolder;
    //Hardware robot = new Hardware();
    private DcMotor leftSide;
    //private DcMotor griper;
    private DcMotor rightSide;
    private DcMotor middleMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        //robot.init(hardwareMap);
        //Set up hardware
        leftSide = hardwareMap.get(DcMotor.class, "left_drive");
        rightSide = hardwareMap.get(DcMotor.class, "right_drive");
        middleMotor =hardwareMap.get(DcMotor.class, "middle_drive");
        //Sets the direction of the motors, one motor as to be reversed for all to spin in same direction
        leftSide.setDirection(DcMotor.Direction.FORWARD);
        rightSide.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.FORWARD);

        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        Location initPos = new Location(0, 0);



        Robot_Prameters rp = new Robot_Prameters( 2500 , new Location(0,0)); // I assumed we use dimensions in millimeters
        Field field  = new Field(400000 , 40000);
        //imu = robot.imu;
        //fundationHolder = robot.fundationHolder;



//
//        AutoDriving AD = new AutoDriving(robot.rightDrive , robot.leftDrive , robot.middleDrive ,
//                 robot.imu,rp,field , telemetry);

        AutoDrivingSecondTry ad = new AutoDrivingSecondTry(leftSide, rightSide, middleMotor, imu, telemetry,this, null,null, new Location(600,0));
        // robot.imu, telemetry, this);

        waitForStart();

        try {
            //  fundationHolder.setPosition(0);
////            ad.setPosition(new Location(200, -100),
////                    -90, 20, 200, 10, 60, 0.6 );



            telemetry.addData("Angle: ", imu.getAngularOrientation());
            //telemetry.update();
            //Orientation angle =  imu.getAngularOrientation();
            int y = 0;

            ad.setPosition(new Location(1175, y),
                    -1, 25, 25, 10, 30, 0.3 );
            //runtime = new ElapsedTime();
            telemetry.addData("At pos: ", 1);
            telemetry.update();
            leftSide.setPower(0);
            rightSide.setPower(0);
            double start = runtime.milliseconds() ;
            //while (runtime.milliseconds()-start < 1000)  { telemetry.addData("time:",runtime.milliseconds()-start ); telemetry.update();}
            //fundationHolder.setPosition(0);
            //Thread.sleep(3000);
            start = runtime.milliseconds() ;
            while (runtime.milliseconds()-start < 1000) {
                telemetry.addData("time:",runtime.milliseconds()-start );
                telemetry.update();}

            //Thread.sleep(1000);
            //coment
            //ad.setPosition(new Location(0, 0),
                    //0, 100, 200, 10, 30, .9 );

            //start = runtime.milliseconds() ;
            //while (runtime.milliseconds()-start < 1000) { }
            Thread.sleep(1000);
            //telemetry.addData("At pos: ", 2);
            //telemetry.update();
            //fundationHolder.setPosition(1);
            //start = runtime.milliseconds() ;
            //while (runtime.milliseconds()-start < 1000) { }
            //Thread.sleep(1000);
            //ad.setPosition(new Location(1175, 2100),
                    //0, 25, 20, 10, 30, 0.5 );
           //ad.setPosition(new Location(1500, y), 0, 50, 25, 10, 0, 0.5 );
            //ad.setPosition(new Location(1500, 1500),
                     //0,25,20,10,30, 0.5);
            //ad.setPosition(new Location(2100, y),
                    //0, 25, 20, 10, 30, 0.5 );
//            ad.setPosition(new Location(500, 2500),
//                    0, 150, 250, 200, 0, 0.5 );
//            ad.setPosition(new Location(-1500, 2500),
//                    0, 150, 250, 200, 0, 0.5 );

            ad.stopAllAutoCalculations();
        }catch (Exception e){ telemetry.addData("error:",e.getStackTrace().toString());}
        runtime = new ElapsedTime();
        //while (runtime.milliseconds() < 10000) { }

//        Location point = new Location( 830 , 420);
//        AD.DriveToCordinate(point , 15 , 0 );
//
//        AD.RotateRobotToAngle(180, 3, 90 ,0.6);
//
//        point = new Location( 500 , 980);
//        AD.DriveToCordinate(point , 15 , 180 );
//
//        fundationHolder.setPosition(1);
//
//        point = new Location( 830 , 220);
//        AD.DriveToCordinate(point , 15 , 180 );
//
//        fundationHolder.setPosition(0);

//        Location point = new Location( -150 , -300);
//        AD.DriveToCordinate(point , 15 , 0 );

//        runtime.reset();
//        while(runtime.milliseconds() < 1000){}
//
//
//        AD.RotateRobotToAngle(-90,4,20,0.6);
//
//
//        point = new Location( -150 , -400);
//        AD.DriveToCordinate(point , 15 , -90 );
//
//
//        runtime.reset();
//        while(runtime.milliseconds() < 1000){}
//
//
//        robot.rightExpantion.setPosition(0);
//
//
//        runtime.reset();
//        while(runtime.milliseconds() < 1000){}
//
//
//        AD.RotateRobotToAngle(0,3,15,1);
//
//        point = new Location( -600 , -400);
//        AD.DriveToCordinate(point , 20 , 0 );
//
//        robot.rightExpantion.setPosition(1);
//
//
//        point = new Location( 1000 , 0);
//        AD.DriveToCordinate(point , 50 , 0 );



    }
}

