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

package org.firstinspires.ftc.teamcode.FIRE_team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="park", group="Pushbot")

public class park extends LinearOpMode  {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private Servo rightExpansion;
    //Hardware robot = new Hardware();
    private DcMotor leftSide;
    //private DcMotor griper;
    private DcMotor rightSide;
    private DcMotor middleMotor;


    @Override
    public void runOpMode() {
        leftSide = hardwareMap.get(DcMotor.class, "left_drive");
        rightSide = hardwareMap.get(DcMotor.class, "right_drive");
        middleMotor =hardwareMap.get(DcMotor.class, "middle_drive");
        rightExpansion = hardwareMap.get(Servo.class,"rightExpansion");
        //Sets the direction of the motors, one motor as to be reversed for all to spin in same direction
        leftSide.setDirection(DcMotor.Direction.FORWARD);
        rightSide.setDirection(DcMotor.Direction.REVERSE);
        middleMotor.setDirection(DcMotor.Direction.FORWARD);

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

        while (runtime.milliseconds()<3000){
            leftSide.setPower(.5);
            rightSide.setPower(.5);
            rightExpansion.setPosition(0.65);
        }
        leftSide.setPower(0);
        rightSide.setPower(0);
        middleMotor.setPower(0);





//        leftSide = robot.leftDrive ;
//        rightSide = robot.rightDrive;
//        middleMotor = robot.middleDrive;


//        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }
}
