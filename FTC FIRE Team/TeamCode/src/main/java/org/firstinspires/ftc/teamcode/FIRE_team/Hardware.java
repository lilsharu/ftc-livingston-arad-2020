package org.firstinspires.ftc.teamcode.FIRE_team;
/**
 * we import imu,DcMotor,distance sensor , hard ware map , servo, elapsed time , JustLoggingAccelerationIntegrator, Position and Velocity
 */

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
    /** Declare  the parts and verbals   */

    BNO055IMU imu;

    public DcMotor  middleDrive = null;
    public DcMotor liftMotor = null;
    public DcMotor leftDrive = null;
    public DcMotor gripperMotor = null;
    public DcMotor rightDrive = null;
    public DcMotor parkingMotor = null;
    public Servo fundationHolder = null;
    public Servo leftExpantion = null;
    public Servo rightExpantion = null;
    public Servo fourbarServo = null;
    public Servo clawServo = null;
    public Servo gripperServo = null;
    public DistanceSensor liftSensor = null;
    public DistanceSensor sideDistanceSensor = null;
    public DistanceSensor frontDistanceSensor = null;


    HardwareMap hwMap =  null;
    private ElapsedTime period = new ElapsedTime();

    /**
     * we set up the HardwareMap
     * @param ahwMap
     */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize imu gyro
/**
 * we set up imu parameters
 */
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
/**
 * we add parts to the hardware map
 */
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        liftMotor = hwMap.get(DcMotor.class,"lift_motor");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        middleDrive = hwMap.get(DcMotor.class, "middle_drive");
        gripperMotor = hwMap.get(DcMotor.class ,"gripper_motor");
        parkingMotor = hwMap.get(DcMotor.class ,"parking_motor");
        rightExpantion = hwMap.get(Servo.class, "right_expansion");
        leftExpantion = hwMap.get(Servo.class, "left_expansion");
        fourbarServo = hwMap.get(Servo.class ,"fourbar-servo");
        clawServo = hwMap.get(Servo.class, "claw_servo");
        gripperServo = hwMap.get(Servo.class ,"gripper_servo");
        liftSensor = hwMap.get(DistanceSensor.class, "lift_sensor");
        fundationHolder = hwMap.get(Servo.class, "foundation_holder");
        frontDistanceSensor = hwMap.get(DistanceSensor.class, "front-distance-sensor");
        sideDistanceSensor = hwMap.get(DistanceSensor.class, "side-distance-sensor");



/**
 * we set up the motors directions
 */
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        middleDrive.setDirection(DcMotor.Direction.REVERSE);
        gripperMotor.setDirection(DcMotor.Direction.REVERSE);

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
/**
 * we set all motors to no power
 */
        leftDrive.setPower(0);
        gripperMotor.setPower(0);
        rightDrive.setPower(0);
        liftMotor.setPower(0);
        middleDrive.setPower(0);
    }
}
