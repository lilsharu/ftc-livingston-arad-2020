package org.firstinspires.ftc.teamcode;

/*
 * The Driver-Controlled code for FireStorm Robotics Team
 * Created by Team Member Aryan Bansal and Ally Mintz
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.*;

@TeleOp(name="Basic TeleOp", group="TeleOp")

public class BasicTeleOp extends OpMode {

    RobotHardware Robot = new RobotHardware();

    //Runs while init is pressed and before play
    public void init() {
        Robot.init(hardwareMap);
        telemetry.addData("Status", "TeleOp has been Initialized");
    }
    public void init_loop() {
        telemetry.addData("Status", "Waiting for Start");
    }

    //runs only once when the driver hits play
    public void start() {
        //runtime.reset();
        telemetry.addData("Status", "Driver Controlled Started");
    }

    //loops after TeleOp Starts
    @Override
    public void loop(){
        //The Joystick:
        //Gamepad 1
        double G1rightStickY = -gamepad1.right_stick_y;
        double G1leftStickY = -gamepad1.left_stick_y;
        double G1rightStickX = -gamepad1.right_stick_x;
        double G1leftStickX = -gamepad1.left_stick_x;

        //Gamepad 2
        double G2rightStickY = -gamepad2.right_stick_y;
        double G2leftStickY = -gamepad2.left_stick_y;
        double G2rightStickX = -gamepad2.right_stick_x;
        double G2leftStickX = -gamepad2.left_stick_x;

        //The Bumpers:
        //Gamepad 1
        boolean G1rightBumper = gamepad1.right_bumper;
        boolean G1leftBumper = gamepad1.left_bumper;

        //Gamepad 2
        boolean G2rightBumper = gamepad2.right_bumper;
        boolean G2leftBumper = gamepad2.left_bumper;

        //The Buttons:
        //Gamepad 1
        boolean G1a = gamepad1.a;
        boolean G1b = gamepad1.b;
        boolean G1x = gamepad1.x;
        boolean G1y = gamepad1.y;
        boolean G1dpadUp = gamepad1.dpad_up;
        boolean G1dpadDown = gamepad1.dpad_down;
        boolean G1dpadLeft = gamepad1.dpad_left;
        boolean G1dpadRight = gamepad1.dpad_right;

        //Gamepad 2
        boolean G2a = gamepad2.a;
        boolean G2b = gamepad2.b;
        boolean G2x = gamepad2.x;
        boolean G2y = gamepad2.y;
        boolean G2dpadUp = gamepad2.dpad_up;
        boolean G2dpadDown= gamepad2.dpad_down;
        boolean G2dpadLeft = gamepad2.dpad_left;
        boolean G2dpadRight = gamepad2.dpad_right;

        double power = Math.sqrt(Math.pow(G1leftStickX, 2) + Math.pow(G1leftStickY, 2));
        double deadZone = 0.1;
        double rightPower =0;
        double leftPower = 0;
        Robot.middle_drive.setPower(0);

        //If the Joystick is approximately aligned to the positive Y axis (therefore forward motion is applied)
        if (Math.abs(G1leftStickX) <= deadZone && G1leftStickY > deadZone) {
            rightPower = power;
            leftPower = power;
        }
        //If the Joystick is approximately aligned to the negative Y axis (therefore backwards motion is applied)
        else if (Math.abs(G1leftStickX) <= deadZone && G1leftStickY < -deadZone) {
            rightPower = -power;
            leftPower = -power;
        }
        //If the Joystick is approximately aligned to the positive X axis (therefore leftwards motion is applied)
        else if (G1leftStickX > deadZone && Math.abs(G1leftStickY) <= deadZone) {
            rightPower = power;
            leftPower = -power;
        }
        //If the Joystick is approximately aligned to the negative X axis (therefore rightwards motion is applied)
        else if (G1leftStickX > deadZone && Math.abs(G1leftStickY) <= deadZone) {
            rightPower = -power;
            leftPower = power;
        }
        //If the Joystick is aligned to the First Quadrant of the coordinate plane (therefore forwards-left motion is aplied)
        else if (G1leftStickX > deadZone && G1leftStickY > deadZone) {
            rightPower = 0.5 * power;
            leftPower = power;
        }
        //If the Joystick is aligned to the Second Quadrant of the coordinate plane (therefore forwards-right motion is aplied)
        else if (G1leftStickX < deadZone && G1leftStickY > deadZone) {
            rightPower = power;
            leftPower = 0.5*power;
        }
        //If the Joystick is aligned to the Third Quadrant of the coordinate plane (therefore backwards-right motion is aplied)
        else if (G1leftStickX < -deadZone && G1leftStickY < -deadZone) {
            rightPower = -power;
            leftPower = -0.5*power;
        }
        //If the Joystick is aligned to the Fourth Quadrant of the coordinate plane (therefore backwards-left motion is aplied)
        else if (G1leftStickX > deadZone && G1leftStickY < deadZone){
            rightPower = -0.5 * power;
            leftPower = -power;
        }
      
        else {
            rightPower = 0;
            leftPower = 0;
            Robot.middle_drive.setPower(0);
        }

        Robot.rightFrontMotor.setPower(rightPower);//replace left stick y with right stick y if you want more control
        Robot.leftFrontMotor.setPower(leftPower);//replace left stick y with right stick y if you want more control


        //Add more stuff as more things are made. This is for a basic drivetrain. We can change the values to accomodate the drivetrain we are working with.
    }
}
