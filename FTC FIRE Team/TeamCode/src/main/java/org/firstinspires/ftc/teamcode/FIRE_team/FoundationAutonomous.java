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

package org.firstinspires.ftc.teamcode;
/**
        *In this section we import  imu ,Autonomous ,LinearOpMode Servo and ElapsedTime
*/

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.FIRE_team.AutoDrivingSecondTry;
import org.firstinspires.ftc.teamcode.FIRE_team.Hardware;
import org.firstinspires.ftc.teamcode.FIRE_team.Location;
import org.firstinspires.ftc.teamcode.FIRE_team.Teleop;

/**
 * we set up the name of the Autonomous
 */
@Autonomous(name="Foundation Autonomous", group="Pushbot")

public class FoundationAutonomous extends LinearOpMode  {

    /** Declare on the parts  */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private Servo fundationHolder;
    private DistanceSensor frontDistanceSensor;
    private DistanceSensor sideDistanceSensor;
    private Servo leftExpantion;
    private DcMotor leftSide;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    @Override

    public void runOpMode() {
/*
 * we init parts from Hardware
 */
        robot.init(hardwareMap);

//        Robot_Prameters rp = new Robot_Prameters( 2500 , new Location(830,220)); // I assumed we use dimensions in millimeters
//        Field field  = new Field(400000 , 40000);
        imu = robot.imu;
        fundationHolder = robot.fundationHolder;
//        sideDistanceSensor = robot.sideDistanceSensor;
//        frontDistanceSensor = robot.frontDistanceSensor;
        leftExpantion = robot.leftExpantion;
        leftSide = robot.leftDrive;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;

        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        AutoDriving AD = new AutoDriving(robot.rightDrive , robot.leftDrive , robot.middleDrive ,
//                 robot.imu,rp,field , telemetry);
/**
 * we send motors imu and distance sensors
 */
        AutoDrivingSecondTry ad = new AutoDrivingSecondTry(robot.leftDrive, robot.rightDrive, robot.middleDrive,
                robot.imu, telemetry, this, frontDistanceSensor, sideDistanceSensor, new Location(0,620));

        waitForStart();
/**
 * we set up the path
 */
        try {

//            ad.setPosition(new Location(800, 450),
//                    -90, 100, 200, 5, 30, 0.8);
//
//
//            runtime = new ElapsedTime();
//            while(runtime.milliseconds() < 500)
//            {
//                leftSide.setPower(0.4);
//                rightSide.setPower(0.4);
//            }
//
//
//            leftSide.setPower(0);
//            rightSide.setPower(0);
//
//
//            runtime = new ElapsedTime();
//            while(runtime.milliseconds() < 500){ }
//
//
//            leftExpantion.setPosition(1);
//            foundationHolder.setPosition(0);
//
//
//            runtime = new ElapsedTime();
//            while(runtime.milliseconds() < 1000){ }
//
//
//            telemetry.addData("while loop began:", true);
//            telemetry.update();
//
//            ad.DriveToWall(-90, 5, 30, 0.5);
//
//
//            telemetry.addData("distance: ", frontDistanceSensor.getDistance(DistanceUnit.MM));
//            telemetry.addData("distances: ", sideDistanceSensor.getDistance(DistanceUnit.MM));
//
//            telemetry.addData("while loop ended:", true);
//            telemetry.update();
//
//            runtime = new ElapsedTime();
//            while(runtime.milliseconds() < 500){
//            }
//
//            foundationHolder.setPosition(1);
//
//            runtime = new ElapsedTime();
//            while(runtime.milliseconds() < 5000){}
//
//
//            //ad.setPosition(new Location(200, 1500),
//              //      -90, 50, 200, 10, 30, 0.8 );
//
//            leftExpantion.setPosition(0);
//
//            ad.stopAllAutoCalculations();

            ad.setPosition(new Location(800, 450),
                    -90, 50, 200, 5, 30, 0.8);


            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500)
            {
                leftSide.setPower(0.4);
                rightSide.setPower(0.4);
            }


            leftSide.setPower(0);
            rightSide.setPower(0);


            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500){ }


            leftExpantion.setPosition(1);
            fundationHolder.setPosition(0);


            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 1000){ }


            ad.DriveToWall(-90, 5, 30, 1, 100);

            //test
            ad.setPosition(new Location(700 ,500 ) ,0, 50 ,200 ,10 ,30 ,0.8);
            ad.setPosition(new Location(700, 400), 0 , 50 ,200 ,10 ,20 ,0.8 );



            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500){
            }

            fundationHolder.setPosition(1);

            runtime = new ElapsedTime();
            while(runtime.milliseconds() < 500){}


            ad.setPosition(new Location(200, 1500),
                    -90, 50, 200, 10, 30, 0.8 );

            leftExpantion.setPosition(0);

            ad.stopAllAutoCalculations();
            Teleop.angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - 90;

        }
        catch (Exception e)
        { telemetry.addData("error:",e.getStackTrace().toString());}


    }
}
