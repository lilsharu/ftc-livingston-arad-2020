
package org.firstinspires.ftc.teamcode;

//import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
//import com.qualcomm.robotcore.hardware.NormalizedRGBA;
//import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;

//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
//import com.qualcomm.robotcore.hardware.SwitchableLight;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcontroller.external.samples.SampleRevBlinkinLedDriver;

@TeleOp(name = "test1.1", group = "TankDrive")
public class test1.1 extends OpMode {

    //Initialize hardware
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    //private Servo frontServo;
    //boolean a1held = false;
    //boolean x1held = false;


    public void init() {
        //Runs when a driver hits "INIT"

        //Initialize hardware variables
        leftFrontMotor = hardwareMap.dcMotor.get("leftFront");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFront");

        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }
    public void start() {
        //Runs when a driver hits "PLAY"
    }

    public void loop() {
        //Runs indefinitely
        boolean buttona1 = gamepad2.a;
       // boolean buttonb1 = gamepad2.b;

        if (buttona1){
            leftFrontMotor.setPower(1);
            rightFrontMotor.setPower(1);
        }

        /*
        if(Math.abs(leftx1) < 0.05) {
            leftx1 = 0;
        }
        if (Math.abs(lefty1) < 0.05) {
            lefty1 = 0;
        }
        if (Math.abs(rightx1) < 0.05) {
            rightx1 = 0;
        }


        //Calculation variables
        double power = Math.sqrt(Math.pow(leftx1, 2) + Math.pow(lefty1, 2)) * 0.8;
        double angle = Math.atan2(leftx1, -lefty1);
        double steering = rightx1;

        if (leftBumper1) {
            steering -= 0.2;
        }
        if (rightBumper1) {
            steering += 0.2;
        }

        //Do advanced trig math (???) and add it to telemetry
        double frontLeft = power*Math.sin(-angle + (Math.PI/4)) - steering;
        telemetry.addData("Front Left: ", frontLeft);
        double frontRight = power*Math.cos(-angle + (Math.PI/4)) + steering;
        telemetry.addData("Front Right: ", frontRight);

        //Apply calculations to motor speed
        leftFrontMotor.setPower(1.5*(frontLeft));
        rightFrontMotor.setPower(1.5*(-frontRight));
       */


        telemetry.update();
    }

    public void stop() {
        // Run once when driver hits "STOP" or time elapses
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
    }

}
