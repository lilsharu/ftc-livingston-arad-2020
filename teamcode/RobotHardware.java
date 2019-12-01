/*
 * The Robot Hardware code for the Livingston-Arad
 * Created by Team Member Aryan Bansal
 */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    //Declaring the specific hardware for our robot
    //The DC Motors:
    private DcMotor middle_drive;
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
/*
    //The Servos
    private Servo liftClawServo;
    private Servo servoTwo;
    private Servo servoThree;
    private Servo servoFour;
*/
    //The Color Sensors
    private ColorSensor colorSensor;

    private static final double SERVO_INIT_POS = 0.5;//This is the initial position of a servo and what you will send it back to
    private static final double SERVO_OPEN_POS = 45;

    HardwareMap hardwareMap;


    public void init(HardwareMap HardwareMap) {
        hardwareMap = HardwareMap;

        //Initializing Motors
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        middle_drive = hardwareMap.get(DcMotor.class, "middle_drive");

        //Set Directions
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE); 
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        middle_drive.setDirection(DcMotor.Direction.FORWARD);


        //Set Motor Power to Zero
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        middle_drive.setPower(0);

        //Run with Encoders. If we don't use Encoders, change "RUN_USING_ENCODERS" to "RUN_WITHOUT_ENCODERS"
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middle_drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        //Brakes the Motors when the power is at Zero
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        middle_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
/*
        //Define and Initialize Servos
        clawServo = hardwareMap.get(Servo.class, "<Servo One Name>");
        servoTwo = hardwareMap.get(Servo.class, "<Servo Two Name>");
        clawServo.setPosition(SERVO_INIT_POS);
        servoTwo.setPosition(SERVO_INIT_POS);
*/
        //Define a Color Sensor
        //Used https://ftc-tricks.com/overview-color-sensor/ to initialize and use Color Sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "Block Detector");
/*
        //Define Gyro Sensor
        gyroSensor = hardwareMap.get(GyroSensor.class, "Gyro Sensor");

        //Define Distance Sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "Distance Sensor");
        */
    }
   
}
