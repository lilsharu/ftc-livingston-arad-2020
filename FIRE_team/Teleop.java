package org.firstinspires.ftc.teamcode;

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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp(name="Basic: Teleop", group="Linear Opmode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members.

    Hardware robot = new Hardware();

    private Robot robotObj;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftSide;
    private DcMotor griper;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    private DcMotor lift_test;

    private DistanceSensor lift_ctrl;
    private Servo griper_servo;
    private final double foundation_height = 66;
    private final double minimalHight = 52;
    private final double level_height= 103;
    private boolean IsPressed = false;
    private double deadZone = 0.1;
    Wheel lift = new Wheel(45,2250,"omni","mm");




    @Override
    public void runOpMode() {
        robotObj = new Robot(2240, hardwareMap, this);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.init(hardwareMap);
        lift_test =robot.lift_test1;
        leftSide = robot.leftDrive ;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;
        griper = robot.griper;
        lift_ctrl = robot.lift;
        griper_servo =robot.griper_servo;



        //Varibles for joystick position
        double G1rightStickY = -gamepad1.right_stick_y;
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1rightStickX = -gamepad1.right_stick_x;
        double G1leftStickX = -gamepad1.left_stick_x;




        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower =0;
        double rightPower =0;
        double middlePower = 0;
        boolean fourbarIsOpen =true;
        boolean graberIsOpen= true;
        boolean rollerGriperIsOpen = true;
        boolean foundation_griperIsOpen = true;
        int level = 0;
        int range = 20;



        // Wait for the game to start (driver presses PLAY)

        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("range", String.format("%.01f mm", lift_ctrl.getDistance(DistanceUnit.MM)));
            double trigger_button = gamepad2.right_trigger;
//
            telemetry.addData( "trigger_button",(trigger_button));

//
//                if( lift_ctrl.getDistance(DistanceUnit.MM)<lift1.High() && gamepad1.a ){
//
//
//                lift_test.setPower(1);
//            }else lift_test.setPower(0);
//
//
//            telemetry.addData("range", String.format("%.01f mm", lift_ctrl.getDistance(DistanceUnit.MM)));


//Would be start of lift method???
            //Change to is pressed?
            //if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_left || gamepad2.dpad_right) { //Should stop lift from moving without dpad being pressed
                if (gamepad2.dpad_up && level < 6 && !IsPressed) { //Lift
                    level++;
                    //IsPressed = true;
                    IsPressed =moveLift(level, range);

                } //else {
                    //IsPressed = false;
                //}

                else if (gamepad2.dpad_down && level > 0) { // && !IsPressed) { //!IsPressed part stops someone from spamming it down
                    level--;
                    //IsPressed = true;
                    IsPressed = moveLift(level, range);
                } //else {
                    //IsPressed = false;
                //}


                if (gamepad2.b) {
                    level = 0;
                }

                //Math to move to lift

                double sample = lift_ctrl.getDistance(DistanceUnit.MM) - (minimalHight + level_height * level);
                double error = Math.abs(sample);


                //Either add condition Ispressed == true or make method
                if (error > range) { //Shouldn't be runable when dpad hasn't been pressed
                    //Stops lift from raising on int if sensor is disconnected
                    if (sample >= 5000) {
                        telemetry.addLine("Distance sensor doesn't work");
                    } else if (sample < 0) {
                        lift_test.setPower(0.8);
                    } else if (sample > 0) {
                        lift_test.setPower(-0.8);
                    }

                } else {
                    lift_test.setPower(0);
                }

           // }
//Would be end of lift method???

            if(gamepad2.back){
                robot.rightExpantion.setPosition(0);
                robot.leftExpantion.setPosition(0);
            }else if (gamepad2.start){
                robot.leftExpantion.setPosition(0.25);
                robot.rightExpantion.setPosition(-0.25);
            }


            rightPower = leftPower  = -gamepad1.left_stick_y ;
            middlePower = -gamepad1.left_stick_x ;
            /*
                if (Math.abs(G1leftStickY) < deadZone && Math.abs(G1leftStickX) < deadZone && Math.abs(G1rightStickX) < deadZone) {
                    robotObj.turnOff();
                }
                else if (Math.abs(G1leftStickY)< deadZone && Math.abs(G1leftStickX)< deadZone && Math.abs(G1rightStickX) >= deadZone) {
                    if (G1rightStickX > 0) {
                        robotObj.turn(G1rightStickX, "r");
                    }
                    else {
                        robotObj.turn(-(G1rightStickX), "l");
                    }
                }
                else if (((Math.abs(G1leftStickY) >= deadZone) || Math.abs(G1leftStickX) >= deadZone) && Math.abs(G1rightStickX) < deadZone) {
                    robotObj.setForwardPower(G1leftStickY);
                    robotObj.strafe(G1leftStickX);
                }
                else {
                    robotObj.setRightPower(rightPower(G1leftStickY, G1rightStickX));
                    robotObj.setLeftPower(leftPower(G1leftStickY, G1rightStickX));
                    robotObj.strafe(G1leftStickX);
                }

            /*
            if(gamepad1.left_bumper)
            {
                rightPower = 0.4;
                leftPower = -0.4;
            }
            else if(gamepad1.right_bumper)
            {
                rightPower = -0.4;
                leftPower = 0.4;
            }

            leftSide.setPower(leftPower);
            rightSide.setPower(rightPower);
            middleMotor.setPower(middlePower);
            */



            if(gamepad2.y && graberIsOpen )
            {
                robot.catchStone.setPosition(0.35);//close
                graberIsOpen = false;
            }
            else if (gamepad2.y && graberIsOpen == false)
            {
                robot.catchStone.setPosition(0.6);//open
                graberIsOpen = true;
            }


            if(gamepad2.a && fourbarIsOpen)
            {
                robot.catchStone.setPosition(0.35);//close
                robot.fourbar.setPosition(0.65);//open
                fourbarIsOpen = false;
            }
            else if (gamepad2.a && fourbarIsOpen == false)
            {
                robot.catchStone.setPosition(0.35);//close
                robot.fourbar.setPosition(0.2);//close
                fourbarIsOpen = true;
            }


            if (gamepad2.x && rollerGriperIsOpen == false)
            {
                griper_servo.setPosition(1);
                rollerGriperIsOpen = true;
            }
            else if (gamepad2.x && rollerGriperIsOpen)
            {
                griper_servo.setPosition(0);
                rollerGriperIsOpen = false;
            }


            if (gamepad2.right_bumper)
            {
                griper.setPower(0.8);
            }
            else if (gamepad2.left_bumper)
            {
                griper.setPower(-0.8);
            }
            else
            {
                griper.setPower(0);
            }


            double cs  = robot.catchStone.getPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData( "a" , robot.leftExpantion.getPosition());
            telemetry.addData( "c" , robot.rightExpantion.getPosition());
            telemetry.addData( "griper-servo" , robot.griper_servo.getPosition());
            telemetry.addData("level: ", level);
            telemetry.addData("lift-ctrl: ", lift_ctrl.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }

    int pos = lift_test.getCurrentPosition();
    double dist = lift.getDistance(pos);




    int maxPos=300;//Change later
    public void ControlLift(double power){
        telemetry.addData("Height", dist);
        int range = 10;
        int intpos = lift_test.getCurrentPosition();
        //Checks if is at max height and num is postive
        if (intpos >= (maxPos-range) && range>0){ //If lift goes higher will go out of bounds
            telemetry.addLine("Lift won't go higher");
        }else if (intpos <= 0 && range<0){//If lift goes lower it will go out of bounds
            telemetry.addLine("Lift won't go lower");
        }else {
            telemetry.addData("init pos:", intpos);

            if (gamepad2.dpad_right){
                lift_test.setPower(power);
            }
            else if (gamepad2.dpad_left){
                lift_test.setPower(-(power));
            }else{
                lift_test.setPower(0);
            }

        }



    }

    public void finalTeleopStrafe(double G1rightStickY, double G1leftStickY, double G1rightStickX, double G1leftStickX){
        if (Math.abs(G1leftStickY) < deadZone && Math.abs(G1leftStickX) < deadZone && Math.abs(G1rightStickX) < deadZone) {
            turnOff();
        }
        else if (Math.abs(G1leftStickY)< deadZone && Math.abs(G1leftStickX)< deadZone && Math.abs(G1rightStickX) >= deadZone) {
            if (G1rightStickX > 0) {
                robotObj.turn(G1rightStickX, "r");
            }
            else {
                robotObj.turn(-(G1rightStickX), "l");
            }
        }
        else if (((Math.abs(G1leftStickY) >= deadZone) || Math.abs(G1leftStickX) >= deadZone) && Math.abs(G1rightStickX) < deadZone) {
            robotObj.setForwardPower(G1leftStickY);
            robotObj.strafe(G1leftStickX);
        }
        else {
            robotObj.setRightPower(rightPower(G1leftStickY, G1rightStickX));
            robotObj.setLeftPower(leftPower(G1leftStickY, G1rightStickX));
            robotObj.strafe(G1leftStickX);
        }

    }

    public double rightPower(double leftY, double rightY){
        return 0.5 * leftY + 0.5 * rightY;
    }

    public double leftPower(double leftY, double rightY) {
        return 0.5 * leftY - 0.5 * rightY;
    }

    public boolean moveLift(int level, int range){
        double sample = lift_ctrl.getDistance(DistanceUnit.MM) - (minimalHight + level_height * level);
        double error = Math.abs(sample);


        //Either add condition Ispressed == true or make method
        while (error > range) { //Shouldn't be runable when dpad hasn't been pressed
            //Stops lift from raising on int if sensor is disconnected
            if (sample >= 5000) {
                telemetry.addLine("Distance sensor doesn't work");
            } else if (sample < 0) {
                lift_test.setPower(0.8);
            } else if (sample > 0) {
                lift_test.setPower(-0.8);
            }

        }
        if (error == range){
            lift_test.setPower(0);
        }
        else {
            lift_test.setPower(0);
        }
        return false;

    }



    /*public void SetUpEncodersForDistance(int distance){
        //Reset encoder values
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Set target position
        leftDrive.setTargetPosition(distance);
        rightDrive.setTargetPosition(distance);
        middleDrive.setTargetPosition(distance);
        //Switch to position mode
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }*/
    public void turnOff(){
        leftSide.setPower(0);
        rightSide.setPower(0);
        middleMotor.setPower(0);
    }
}
