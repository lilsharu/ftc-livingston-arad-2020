/**
 * Created by Ally on 12/20/2019.
 */

//This file is meant to test functions of the code in isolation
    //Current use: proportionDrive
package org.firstinspires.ftc.teamcode;

//Imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

//Setting up class
@Autonomous(name="testFile", group="Auton")

public class testFile extends LinearOpMode {
    //Setting up hardware
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftSide;
    private DcMotor griper;
    private DcMotor rightSide;
    private DcMotorEx middleMotor;
    private DcMotor lift_test;
    private BNO055IMU imu;

    //private ElapsedTime runtime = new ElapsedTime();
//OpMode code
    @Override
    public void runOpMode() {
        runtime.reset();
        //More set up
        robot.init(hardwareMap);
        Robot_Prameters rp = new Robot_Prameters( 250 , new Location(0,0)); // I assumed we use dimensions in millimeters
        Field field  = new Field(3657.6 , 3657.6);
        AutoDriving AD = new AutoDriving(robot.leftDrive , robot.rightDrive ,robot.middleDrive ,
                robot.imu,rp,field , telemetry);

        waitForStart();


        Wheel mid_drive = new Wheel(45,2240); //Creates a Wheel object

        rightSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //Should this go first?
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Main code starts here
            //Declares distance, gets and uses encoder pos to find position of bot based on it's movement
        double goal = 3000;
        int tickpos = middleMotor.getCurrentPosition();
        double pos = mid_drive.getDistance(tickpos); //Note: as this is a snipit where ever the bot starts is 0.
        double initpos=pos; //Holds original pos to compare to later
        double moved; //Will hold how far the bot has gone
        //Following runs until the robot has moved to its goal
        while (Math.abs(goal-pos)>=5){ //5mm error range
            moved = pos-initpos; //Calculates how much moved from start
            middleMotor.setVelocity(AD.proportionDrive(moved,.1,goal));
            pos = mid_drive.getDistance(tickpos); //I believe that since the encoders haven't reset this will give the total ticks from start
        }
        middleMotor.setPower(0);


    }
    /*
    Notes:
        * Currently only left and middle motors are switched to DcMotorEx
        * Velocity is measured in ticks per a second
        * Formula doesn't account for friction as of now
        * Files needed to be imported to test: This, Wheel.java, AutoDriving, Field, RobotParamters.java & Hardware
     */

}
