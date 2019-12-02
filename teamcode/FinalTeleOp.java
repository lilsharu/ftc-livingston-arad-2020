/*
 *The FinalTeleOp code for FIRE Team
 * Written by Team Members Shourya Bansal, Ally Mintz, and Pierce Rubenstein
 */
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FinalTeleOp", group="TeleOp")

public class FinalTeleOp extends OpMode {
    //Creates a Robot Class
    private Robot robot = new Robot(2240);
    private HardwareMap hardwareMap;
    ElapsedTime runtime = new ElapsedTime();

    //The controller variables:
    //The Joystick:
    //Gamepad 1
    private double G1rightStickY = -gamepad1.right_stick_y;
    private double G1leftStickY = -gamepad1.left_stick_y;
    private double G1rightStickX = -gamepad1.right_stick_x;
    private double G1leftStickX = -gamepad1.left_stick_x;

    //Gamepad 2
    private double G2rightStickY = -gamepad2.right_stick_y;
    private double G2leftStickY = -gamepad2.left_stick_y;
    private double G2rightStickX = -gamepad2.right_stick_x;
    private double G2leftStickX = -gamepad2.left_stick_x;

    //The Bumpers:
    //Gamepad 1
    private boolean G1rightBumper = gamepad1.right_bumper;
    private boolean G1leftBumper = gamepad1.left_bumper;

    //Gamepad 2
    private boolean G2rightBumper = gamepad2.right_bumper;
    private boolean G2leftBumper = gamepad2.left_bumper;

    //The Triggers:
    //Gamepad 1:
    private double G1rightTrigger = gamepad1.right_trigger;
    private double G1leftTrigger = gamepad1.left_trigger;

    //Gamepad 2:
    private double G2rightTrigger = gamepad2.right_trigger;
    private double G2leftTrigger = gamepad2.left_trigger;

    //The Buttons:
    //Gamepad 1
    private boolean G1a = gamepad1.a;
    private boolean G1b = gamepad1.b;
    private boolean G1x = gamepad1.x;
    private boolean G1y = gamepad1.y;
    private boolean G1dpadUp = gamepad1.dpad_up;
    private boolean G1dpadDown = gamepad1.dpad_down;
    private boolean G1dpadLeft = gamepad1.dpad_left;
    private boolean G1dpadRight = gamepad1.dpad_right;

    //Gamepad 2
    private boolean G2a = gamepad2.a;
    private boolean G2b = gamepad2.b;
    private boolean G2x = gamepad2.x;
    private boolean G2y = gamepad2.y;
    private boolean G2dpadUp = gamepad2.dpad_up;
    private boolean G2dpadDown= gamepad2.dpad_down;
    private boolean G2dpadLeft = gamepad2.dpad_left;
    private boolean G2dpadRight = gamepad2.dpad_right;

    double power = Math.sqrt(Math.pow(G1leftStickX, 2) + Math.pow(G1leftStickY, 2));
    private double deadZone = 0.1;
    private double leftPower;
    private double rightPower;
    private double centerPower;

    public void init(){
        robot.init(hardwareMap);
        telemetry.addData("Status", "The Robot has been Initialized");
        telemetry.update();
        robot.setRunMode("encoders");
        telemetry.addData("RunMode", "RUN_USING_ENCODER");
        telemetry.update();
    }

    public void start(){
        runtime.reset();
        telemetry.addData("Status", "Starting to Move. TeleOp has begun");
        telemetry.update();
    }

    public void loop() {
        if (Math.abs(G1leftStickY) < deadZone && Math.abs(G1leftStickX) < deadZone && Math.abs(G1rightStickX) < deadZone) {
            robot.turnOff();
        }
        else if (Math.abs(G1leftStickY)< deadZone && Math.abs(G1leftStickX)< deadZone && Math.abs(G1rightStickX) >= deadZone) {
            if (G1rightStickX > 0) robot.turn(G1rightStickX, "r");
            else robot.turn(G1rightStickX, "l");
        }
        else if ((Math.abs(G1leftStickY) >= deadZone) || Math.abs(G1leftStickX) >= deadZone && Math.abs(G1rightStickX) < deadZone) {
            robot.setForwardPower(G1leftStickY);
            robot.strafe(G1leftStickX);
        }
        else {
            robot.setRightPower(rightPower(G1leftStickY, G1rightStickX));
            robot.setLeftPower(leftPower(G1leftStickY, G1rightStickX));
            robot.strafe(G1leftStickX);
        }

    }

    //this takes into account turning and everything. I am not sure if this wil work (We'll need to test it)
    public double rightPower(double leftY, double rightY){
        return 0.5 * leftY + 0.5 * rightY;
    }

    public double leftPower(double leftY, double rightY) {
        return 0.5 * leftY - 0.5 * rightY;
    }
}
