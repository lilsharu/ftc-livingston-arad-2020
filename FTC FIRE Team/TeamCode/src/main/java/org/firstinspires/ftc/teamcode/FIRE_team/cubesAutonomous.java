package org.firstinspires.ftc.teamcode.FIRE_team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Cubes Autonomous", group="Pushbot")


public class cubesAutonomous extends LinearOpMode {

    /**
     * Declare on the parts
     */
    Hardware robot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private Servo fundationHolder;
    private Servo gripperServo;
    private DistanceSensor frontDistanceSensor;
    private DistanceSensor sideDistanceSensor;
    private Servo leftExpantion;
    private DcMotor leftSide;
    private DcMotor rightSide;
    private DcMotor middleMotor;
    private DcMotor gripperMotor;
    private double stoneSize = 203.2;
    private int mode = 1;
    private double fieldSide = 3660;
    private double robotLength = 450;
    private double robotWidth = 430;
    private double robotWidthWhenOpen = 550;
    private double cubePostion = 0;

    @Override

    public void runOpMode() {
/**
 * we init parts from Hardware
 */
        robot.init(hardwareMap);

//        Robot_Prameters rp = new Robot_Prameters( 2500 , new Location(830,220)); // I assumed we use dimensions in millimeters
//        Field field  = new Field(400000 , 40000);
        imu = robot.imu;
        fundationHolder = robot.foundationHolder;
        sideDistanceSensor = robot.sideDistanceSensor;
        frontDistanceSensor = robot.frontDistanceSensor;
        leftExpantion = robot.leftExpantion;
        leftSide = robot.leftDrive;
        rightSide = robot.rightDrive;
        middleMotor = robot.middleDrive;
        gripperMotor = robot.gripperMotor;
        gripperServo = robot.gripperServo;
        rightSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        AutoDriving AD = new AutoDriving(robot.rightDrive , robot.leftDrive , robot.middleDrive ,
//                 robot.imu,rp,field , telemetry);
/**
 * we send motors imu and distance sensors
 */
        AutoDrivingSecondTry ad = new AutoDrivingSecondTry(robot.leftDrive, robot.rightDrive, robot.middleDrive,
                robot.imu, telemetry, this, frontDistanceSensor, sideDistanceSensor,
                new Location(0, 2600));

        waitForStart();
/**
 * we set up the path
 */
        try {

            cubePostion = fieldSide - (mode + 4) * stoneSize + robotLength/2;

            ad.setPosition(new Location(600, cubePostion),
                    30, 50, 200, 5, 30, 0.6);

            gripperServo.setPosition(0);

            runtime.reset();
            while(runtime.milliseconds() < 1000)
            {
                leftSide.setPower(0.4);
                rightSide.setPower(0.4);
                gripperMotor.setPower(0.8);
            }


            leftSide.setPower(0);
            rightSide.setPower(0);


            ad.setPosition(new Location(0, 2600),
                    180, 50, 200, 5, 30, 0.8);


            ad.setPosition(new Location(0, 1800),
                    180, 50, 200, 5, 30, 0.8);


            gripperMotor.setPower(-0.8);


        } catch (Exception e) {
            telemetry.addData("error:", e.getStackTrace().toString());
        }
    }
}
