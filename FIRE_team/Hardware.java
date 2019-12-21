package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;


import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

public class Hardware{

    BNO055IMU imu ;


    public DcMotorEx  middleDrive     = null;
    public DcMotor lift_test1 = null;
    public DcMotorEx leftDrive = null;
    public DcMotor griper = null;
    public DcMotor rightDrive = null;

    public Servo    leftExpantion    = null;
    public Servo    fourbar = null;
    public Servo    catchStone = null;
    public Servo    rightExpantion   = null;
    public Servo    griper_servo = null;
    //public DistanceSensor lift ;

    public static final double startPosition = 0;
    public static final double finalPosition = 0.25;

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize imu gyro

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        // Define and Initialize Motors
        leftDrive  = (DcMotorEx)hwMap.get(DcMotor.class, "left_drive");
        lift_test1 = hwMap.get(DcMotor.class,"lift_test");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        middleDrive = (DcMotorEx)hwMap.get(DcMotor.class, "middle_drive");
        griper = hwMap.get(DcMotor.class ,"gripper");

        rightExpantion = hwMap.get(Servo.class, "right_expansion");
        leftExpantion = hwMap.get(Servo.class, "left_expansion");
        fourbar = hwMap.get(Servo.class ,"fourbar");
        catchStone = hwMap.get(Servo.class, "catch_Servo");
        griper_servo = hwMap.get(Servo.class ,"gripper_servo");

        //lift = hwMap.get(DistanceSensor.class, "lift_ctrl");



        lift_test1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleDrive.setDirection(DcMotorEx.Direction.FORWARD);
        griper.setDirection(DcMotor.Direction.REVERSE);

        //Set up enocoders for lift
        lift_test1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_test1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to zero power
        leftDrive.setPower(0);
        griper.setPower(0);
        rightDrive.setPower(0);
        lift_test1.setPower(0);
        middleDrive.setPower(0);
    }




}
