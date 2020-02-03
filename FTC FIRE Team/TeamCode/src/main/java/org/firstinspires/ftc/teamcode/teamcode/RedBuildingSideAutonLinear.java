package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="RedBuildingSideAutonLinear", group="Auton")

public class RedBuildingSideAutonLinear extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private int counter = 0;

    @Override
    public void runOpMode() {
        // Declare all motors
        DcMotor leftDrive = null;
        DcMotor rightDrive = null;
        DcMotor middleDrive = null;
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        leftDrive = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightDrive = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        middleDrive = hardwareMap.get(DcMotor.class, "middle_drive");

        //Set up Encoders
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);

        int wheelticks = 2240;
        //Set up wheels
        Wheel midWheel = new Wheel(45, 1120, "middle", "Omni", "mm");
        Wheel leftWheel = new Wheel(45, wheelticks, "left", "Omni", "mm");
        Wheel rightWheel = new Wheel(45, wheelticks, "right", "Omni", "mm");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        Robot robot = new Robot(rightDrive, leftDrive, middleDrive, rightWheel, leftWheel, midWheel, 2240);


        //Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0; //=leftDrive.getPower();
        double rightPower = 0;//=rightDrive.getPower();
        //double middlePower=middleDrive.getPower();
        //Wheel midWheel = new Wheel(middleDrive,3.5,1400,"Middle");


        double middlePower = 0.8; //Add this to the class?
        //Drive forward
        //Grab foundation
        //Drive backwards
        //int counter = 0;
        robot.moveSideways(middlePower, 24);//Parks

        telemetry.addData("parked", counter);
        counter += 1;
        robot.turnOff();
        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f), middle (%.2f)", leftPower, rightPower, middlePower);
    }
}
