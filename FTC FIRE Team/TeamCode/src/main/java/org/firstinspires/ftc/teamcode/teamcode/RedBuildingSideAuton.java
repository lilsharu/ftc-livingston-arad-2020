package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


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

/**
 * The Basic Auton code for FireStorm Robotics
 * Created by Team Member Ally Mintz
 */

@Autonomous(name="RedBuildingSideAuton", group="Auton")

public class RedBuildingSideAuton extends OpMode{

    // Declare all motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor middleDrive= null;

    //Declare servos

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        middleDrive = hardwareMap.get(DcMotor.class, "middle_drive");

        //Set up Enccoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        middleDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        middleDrive.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower=leftDrive.getPower();
        double rightPower=rightDrive.getPower();
        double middlePower=middleDrive.getPower();
        Wheel midWheel = new Wheel(middleDrive,3.5,1400,"Middle");


        middlePower = 0.8; //Add this to the class?
        int numOfTicks= midWheel.getNumOfTicks();

        moveSideways(middlePower, numOfTicks);
        //Thread.sleep(3000);
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    public void moveSideways(double power, int distance){
        SetUpEncodersForDistance(distance);
        //set drive power
        middleDrive.setPower(power);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        //Has motors run until position is reached
        while(middleDrive.isBusy()){
            //Waits
        }

        //Stops driving
        turnOff();
        //Changes mode back to normal
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        middleDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetUpEncodersForDistance(int distance){
        //Reset encoder values
        leftDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        middleDrive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //Set target position
        leftDrive.setTargetPosition(distance);
        rightDrive.setTargetPosition(distance);
        middleDrive.setTargetPosition(distance);
        //Switch to position mode
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middleDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void moveRight(double power){
        rightDrive.setPower(power);
        leftDrive.setPower(0);
        middleDrive.setPower(0);
    }
    //Could use this instead of individual turning methods
    public void turnOff(){
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        middleDrive.setPower(0);
    }

}



