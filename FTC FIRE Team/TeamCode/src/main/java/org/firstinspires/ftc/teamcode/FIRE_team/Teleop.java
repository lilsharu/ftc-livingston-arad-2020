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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name="Basic: Teleop", group="Linear Opmode")
public class Teleop extends LinearOpMode {

    /**
     *     Declare parts from the Hardware class
     *     and create variable
      */

    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftSide;
    private DcMotor griper;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    private DcMotor lift_test;
    private DcMotor parkingMotor;
    private BNO055IMU imu;
    private DistanceSensor lift_ctrl;
    private Servo griper_servo;
    private Servo fundationHolder;
    volatile boolean g2StartButtonIsPressed = false;
    volatile boolean g1StartButtonIsPressed = false;
    volatile boolean g1BackButtonIsPressed = false;
    volatile boolean isClawPressed;
    volatile boolean isGripperPressed;
    volatile boolean isFourbarPressed;
    volatile boolean isLiftPressed;
    boolean rollerGriperIsOpen = true;
    boolean fourbarIsOpen = true;
    boolean graberIsOpen = true;
    double powerY;
    double powerX;
    double gyroAngle;
    double leftPower;
    double rightPower;
    double middlePower;
    double minimalHight = 20;
    double foundationHight = minimalHight + 50;
    double level_height = 100;
    double sample;
    double error;
    int range = 10;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
/**
 * init the components
 */
        robot.init(hardwareMap);
        lift_test = robot.lift_test1;
        leftSide = robot.leftDrive;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;
        griper = robot.griper;
        lift_ctrl = robot.lift;
        griper_servo = robot.griper_servo;
        fundationHolder = robot.foundationHolder;
        imu = robot.imu;
        parkingMotor = robot.parkingMotor;
/**
 * create more variables
 */
        boolean manualControl = false;
        boolean fieldOrientatedDrive = false;
        boolean foundationHolderIsOpened = false;
        int level = 0;
        gyroAngle = 0;
        isClawPressed = false;
        isFourbarPressed = false;
        isGripperPressed = false;
        isLiftPressed = false;
        waitForStart();
        runtime.reset();
/** create/init  the threads for the teleop
 *
 */
        ActiveLocation activeLocation = new ActiveLocation(leftSide, rightSide, middleMotor,imu , new Location(0,0));
        Thread currentLocationThread = new Thread(activeLocation);
        currentLocationThread.start();

//        DistanceToTargetFinder distanceToTargetFinder = new DistanceToTargetFinder(activeLocation, imu);
//        Thread targetLocationThread = new Thread(distanceToTargetFinder);
//        targetLocationThread.start();

        Location point = new Location(1000, 1000);

//        distanceToTargetFinder.setNewPoint(point);
/**
 * sets the mode for the motors
 */
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /**
             * switch between  of the drive train (field Orientated Drive and normal)
             */
            if (gamepad1.back && fieldOrientatedDrive && !g1BackButtonIsPressed) {
                fieldOrientatedDrive = false;
                g1BackButtonIsPressed = true;
            } else if (gamepad1.back && !fieldOrientatedDrive && !g1BackButtonIsPressed) {
                fieldOrientatedDrive = true;
                g1BackButtonIsPressed = true;
            } else if(!gamepad1.back){
                g1BackButtonIsPressed = false;
            }

            gyroAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            /**
             * the different calculations of the  different modes
             */
            if (fieldOrientatedDrive) {
                powerY = -(gamepad1.left_stick_x * Math.sin(Math.toRadians(-gyroAngle))) + gamepad1.left_stick_y * Math.cos(Math.toRadians(-gyroAngle));
                powerX = gamepad1.left_stick_x * Math.cos(Math.toRadians(-gyroAngle)) + gamepad1.left_stick_y * Math.sin(Math.toRadians(-gyroAngle));


                rightPower = leftPower = powerY;
                middlePower = powerX;
            } else {
                rightPower = leftPower = gamepad1.left_stick_y;
                middlePower = gamepad1.left_stick_x;
            }
/**
 *  turning to both sids
 */
            if (gamepad1.left_trigger > 0) {
                rightPower = -gamepad1.left_trigger;
                leftPower = gamepad1.left_trigger;
                rightPower = Range.clip(rightPower, -0.6, 0.6);
                leftPower = Range.clip(leftPower, -0.6, 0.6);
            } else if (gamepad1.right_trigger > 0) {
                rightPower = gamepad1.right_trigger;
                leftPower = -gamepad1.right_trigger;
                rightPower = Range.clip(rightPower, -0.6, 0.6);
                leftPower = Range.clip(leftPower, -0.6, 0.6);
            }
/**
 * sets power
 */
            leftSide.setPower(leftPower);
            rightSide.setPower(rightPower);
            middleMotor.setPower(middlePower);
/**
 * the opening and closing fourbar of the claw
 */
            if (gamepad2.a && !fourbarIsOpen && !isFourbarPressed) {
                robot.fourbar.setPosition(0);
                fourbarIsOpen = true;
                isFourbarPressed = true;
            } else if (gamepad2.a && fourbarIsOpen) {
                robot.fourbar.setPosition(1);
                fourbarIsOpen = false;
                isFourbarPressed = true;
            }    else if (!gamepad2.a) {
            isFourbarPressed = false;
        }


/**
 * switch between the different  mods of the lift (levels and manual)
 */
            if (gamepad2.start && !g2StartButtonIsPressed && !manualControl) {
                manualControl = true;
                g2StartButtonIsPressed = true;
            } else if (gamepad2.start && !g2StartButtonIsPressed && manualControl) {
                manualControl = false;
                g2StartButtonIsPressed = true;
            } else if(!gamepad2.start){
                g2StartButtonIsPressed = false;
            }

/**
 * move the lift up
 */
            if (gamepad2.dpad_up && level < 6 && !isLiftPressed && !manualControl) {
                level++;
                isLiftPressed = true;
                liftAutoControl(manualControl, level);
            } else if (gamepad2.dpad_up && !isLiftPressed && manualControl) {
                isLiftPressed = true;
                lift_test.setPower(1);
            } else if (!gamepad2.dpad_up) {
                isLiftPressed = false;
            }
/**
 * move the  lift down
 */

            if (gamepad2.dpad_down && level > 0 && !isLiftPressed && !manualControl) {
                level--;
                isLiftPressed = true;
                liftAutoControl(manualControl, level);
            } else if (gamepad2.dpad_down && manualControl) {
                lift_test.setPower(-1);
            } else if (!gamepad2.dpad_down) {
                isLiftPressed = false;
            }
/**
 * stops the lift
 */
            if (manualControl && !gamepad2.dpad_down && !gamepad2.dpad_up)
            {
                lift_test.setPower(0);
            }

            if (gamepad2.b && !manualControl) {
                level = 0;
            }


/**
 * opening and closing the expantion
 */
            if (gamepad1.left_bumper) {
                robot.rightExpantion.setPosition(0);
                robot.leftExpantion.setPosition(1);
            } else if (gamepad1.right_bumper) {
                robot.leftExpantion.setPosition(0);
                robot.rightExpantion.setPosition(1);
            }
/**
 *opening and closing the roller griper
 */
            if (gamepad2.x && rollerGriperIsOpen && !isGripperPressed) {
                griper_servo.setPosition(1);//close
                rollerGriperIsOpen = false;
                isGripperPressed = true;
            } else if (gamepad2.x && !rollerGriperIsOpen && !isGripperPressed) {
                griper_servo.setPosition(0.38);
                rollerGriperIsOpen = true;
                isGripperPressed = true;
            } else if (!gamepad2.x) {
                isGripperPressed = false;
            }
/**
 * spine the intake
 */

            if (gamepad2.right_bumper) {
                griper.setPower(0.8);
            } else if (gamepad2.left_bumper) {
                griper.setPower(-0.8);
            } else {
                griper.setPower(0);
            }
/**
 * open and close the foundationHolder
 */
            if (gamepad1.start && !foundationHolderIsOpened && !g1StartButtonIsPressed) {
                fundationHolder.setPosition(0);
                foundationHolderIsOpened = true;
                g1StartButtonIsPressed = true;
            } else if (gamepad1.start && foundationHolderIsOpened && !g1StartButtonIsPressed) {
                fundationHolder.setPosition(1);
                foundationHolderIsOpened = false;
                g1StartButtonIsPressed = true;
            } else {
                g1StartButtonIsPressed = false;
            }
/**
 * open and close the claw
 */
            if (gamepad2.y && graberIsOpen && !isClawPressed) {
                robot.catchStone.setPosition(0.36);//close
                graberIsOpen = false;
                isClawPressed = true;
            } else if (gamepad2.y && graberIsOpen == false && !isClawPressed) {
                robot.catchStone.setPosition(0.6);//open
                graberIsOpen = true;
                isClawPressed = true;
            } else if (!gamepad2.y) {
                isClawPressed = false;
            }

/**
 * open and close the parkingMotor
 */
            if (gamepad1.dpad_up) {
                parkingMotor.setPower(1);
            } else if (gamepad1.dpad_down) {
                parkingMotor.setPower(-1);
            } else {
                parkingMotor.setPower(0);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f), middle (%.2f)", leftPower, rightPower, middlePower);
            telemetry.addData("angle", gyroAngle);
            telemetry.addData("filed-orientated: ", fieldOrientatedDrive);
            telemetry.addData("currentPoint: ","(x:(%.2f), y:(%.2f))", activeLocation.getX_Axis(), activeLocation.getY_Axis());
            telemetry.addData("level: ", level);
//            double[] distances = distanceToTargetFinder.getDistanceTotarget();
//            telemetry.addData("distances - ","distance in x axis: (%.2f), distance in y axis: (%.2f)", distances[0], distances[1]);
            telemetry.addData("encoders: ","left((%.2f)), right((%.2f)), middle((%.2f))",
                    convertToTicksY(leftSide.getCurrentPosition()), convertToTicksY(rightSide.getCurrentPosition()), convertToTicksX(middleMotor.getCurrentPosition()));
            telemetry.addData("lift-ctrl: ", lift_ctrl.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
        activeLocation.setStop(true);
//        distanceToTargetFinder.setStop(true);
    }

    /**
     * move the lift between levels
     * @param manualControl
     * @param level
     */

    public void liftAutoControl(boolean manualControl, int level)
    {
        if (!manualControl)
        {
            if (level == 0)
            {
                sample = lift_ctrl.getDistance(DistanceUnit.MM) - minimalHight;
            }
            else
            {
                sample = lift_ctrl.getDistance(DistanceUnit.MM) - (foundationHight + level_height * (level - 1));
            }

            error = Math.abs(sample);

            if (error > range)
            {
                if (sample < 0)
                {
                    lift_test.setPower(1);
                }
                else if (sample > 0)
                {
                    lift_test.setPower(-1);
                }
            }
            else
            {
                lift_test.setPower(0);
            }
        }
    }

    /**
     * convert distance to ticks (x axis)
     * @param distance
     * @return ticks in the robot x axis
     */
    public static double convertToTicksX(double distance){
        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return ((distance * DRIVE_GEAR_REDUCTION  * 28.5) / (WHEEL_DIAMETER_MM * Math.PI ));
    }

    /**
     * convert distance to ticks (y axis)
     * @param distance
     * @return ticks in the robot y axis
     */
    private double convertToTicksY(double distance){

        double     DRIVE_GEAR_REDUCTION    = 20 ;
        double     WHEEL_DIAMETER_MM   = 90 ;
        return -((distance * DRIVE_GEAR_REDUCTION  * 28.5)/ (WHEEL_DIAMETER_MM * Math.PI));

    }
}
