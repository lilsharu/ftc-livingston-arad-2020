package org.firstinspires.ftc.teamcode.FIRE_team;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    BNO055IMU imu;

    public DcMotor  middleDrive = null;
    public DcMotor lift_test1 = null;
    public DcMotor leftDrive = null;
    public DcMotor griper = null;
    public DcMotor rightDrive = null;
    public DcMotor parkingMotor = null;
    public Servo fundationHolder = null;
    public Servo leftExpantion = null;
    public Servo fourbar = null;
    public Servo catchStone = null;
    public Servo rightExpantion = null;
    public Servo griper_servo = null;
    public DistanceSensor lift = null;

    public static final double startPosition = 0;
    public static final double finalPosition = 0.25;

    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();


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
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        lift_test1 = hwMap.get(DcMotor.class,"lift_test");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        middleDrive = hwMap.get(DcMotor.class, "middle_drive");
        griper = hwMap.get(DcMotor.class ,"gripper");
        parkingMotor = hwMap.get(DcMotor.class ,"parking_motor");
        rightExpantion = hwMap.get(Servo.class, "right_expansion");
        leftExpantion = hwMap.get(Servo.class, "left_expansion");
        fourbar = hwMap.get(Servo.class ,"fourbar");
        catchStone = hwMap.get(Servo.class, "catch_Servo");
        griper_servo = hwMap.get(Servo.class ,"gripper_servo");
        lift = hwMap.get(DistanceSensor.class, "lift_ctrl");
        fundationHolder = hwMap.get(Servo.class, "foundation_holder");


        lift_test1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);
        griper.setDirection(DcMotor.Direction.REVERSE);

//        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        middleDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set all motors to zero power
        leftDrive.setPower(0);
        griper.setPower(0);
        rightDrive.setPower(0);
        lift_test1.setPower(0);
        middleDrive.setPower(0);
    }
}
