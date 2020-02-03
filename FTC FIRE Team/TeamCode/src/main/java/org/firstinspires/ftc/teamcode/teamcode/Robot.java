package org.firstinspires.ftc.teamcode.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Auton.Wheel;

/*
 * The Specific Robot Class for FIRE
 * Created by Shourya Bansal
 */
public class Robot {
    private DcMotor rightMotor;
    private DcMotor leftMotor;
    private DcMotor middleMotor;
    private DcMotor elevatorMotor;

    //Creates Corresponding Wheels
    private Wheel rightWheel;
    private Wheel leftWheel;
    private Wheel middleWheel;

    //Creates Sensor to Direct
    private ColorSensor leftColor;
    private ColorSensor rightColor;

    //Creates Servos which are being used
    private Servo clawServo;
    private Servo rightServo;
    private Servo leftServo;

    public static final double SERVO_INIT_POS = 0;
    public static final double SERVO_OPEN_POS = 1;

            //Creates an array to store the robot's position after every movement
            double[] position = new double[2];

            //Creates a timer to calculate elapsed time
            ElapsedTime runtime = new ElapsedTime();

    //Two color sensors
    private ColorSensor leftSensor;
    private ColorSensor rightSensor;

    //Variables and values which we need to know
    private double ticksPerRotationForward;
    private double ticksPerRotationSideways;

    //Constructors to create the Robot
    public Robot(DcMotor rightMotor, DcMotor leftMotor, DcMotor middleMotor, DcMotor elevator, Wheel rightWheel, Wheel leftWheel, Wheel middleWheel, ColorSensor leftColor, ColorSensor rightColor, Servo clawServo, Servo rightPlateServo, Servo leftPlateServo, double ticksPerRotation) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.middleMotor = middleMotor;
            this.elevatorMotor = elevator;
            this.rightWheel = rightWheel;
            this.leftWheel = leftWheel;
            this.middleWheel = middleWheel;
            this.leftColor = leftColor;
            this.rightColor = rightColor;
            this.clawServo = clawServo;
            this.rightServo = rightPlateServo;
            this.leftServo = leftPlateServo;
            this.ticksPerRotationForward = ticksPerRotation;
            this.ticksPerRotationSideways = ticksPerRotation * 0.5;
            }
    public Robot(DcMotor rightMotor, DcMotor leftMotor, DcMotor middleMotor, Wheel rightWheel, Wheel leftWheel, Wheel middleWheel, double ticksPerRotation) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.middleMotor = middleMotor;
            this.rightWheel = rightWheel;
            this.leftWheel = leftWheel;
            this.middleWheel = middleWheel;
            this.ticksPerRotationForward = ticksPerRotation;
            this.ticksPerRotationSideways = 0.5 * ticksPerRotation;
            }
    public Robot(DcMotor rightMotor, DcMotor leftMotor, DcMotor middleMotor, double ticksPerRotation) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.middleMotor = middleMotor;
            this.rightWheel = new Wheel(96, 2240, "Right Wheel", "Omni", "mm");
            this.leftWheel = new Wheel(96, 2240, "Left Wheel", "Omni", "mm");
            this.middleWheel = new Wheel(96, 1120, "Middle Wheel", "Omni", "mm");
            this.ticksPerRotationForward = ticksPerRotation;
            this.ticksPerRotationSideways = 0.5 * ticksPerRotation;
            }
    public Robot(double ticksPerRotation, HardwareMap hardwareMap) {
            ticksPerRotationForward = ticksPerRotation;
            ticksPerRotationSideways = 0.5 * ticksPerRotation;
            this.rightWheel = new Wheel(96, Convert.round(ticksPerRotationForward), "Right Wheel", "Omni", "mm");
            this.leftWheel = new Wheel(96, Convert.round(ticksPerRotationForward), "Left Wheel", "Omni", "mm");
            this.middleWheel = new Wheel(96, Convert.round(ticksPerRotationSideways), "Middle Wheel", "Omni", "mm");
            this.leftMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
            this.rightMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
            this.middleMotor = hardwareMap.get(DcMotor.class, "middle_drive");

            //this is guess and test, change if necessary
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            middleMotor.setDirection(DcMotor.Direction.REVERSE);
            //elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
            turnOff();//Sets all motor powers to zero
            //Runs Using Encoders (The Standard)
            setRunMode("using");
            //brakes the motors and sets active resistance when power is zero
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

    //Initializes Robot

    public void init(HardwareMap aHardwareMap) {
            leftMotor = aHardwareMap.get(DcMotor.class, "leftFrontMotor");
            rightMotor = aHardwareMap.get(DcMotor.class, "rightFrontMotor");
            middleMotor = aHardwareMap.get(DcMotor.class, "middle_drive");
            //elevatorMotor = hardwareMap.get(DcMotor.class, "Claw Motor");

            //this is guess and test, change if necessary
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            middleMotor.setDirection(DcMotor.Direction.REVERSE);
            //elevatorMotor.setDirection(DcMotor.Direction.FORWARD);

            turnOff();//Sets all motor powers to zero
            //Runs Using Encoders (The Standard)
            setRunMode("using");

            //brakes the motors and sets active resistance when power is zero
            leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            middleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            /*
            //Now Servos
            clawServo = hardwareMap.get(Servo.class, "Claw Servo");
            rightServo = hardwareMap.get(Servo.class, "Right Servo");
            leftServo = hardwareMap.get(Servo.class, "Left Servo");

            clawServo.setPosition(SERVO_INIT_POS);
            leftServo.setPosition(SERVO_INIT_POS);
            rightServo.setPosition(SERVO_INIT_POS);

            leftColor = hardwareMap.get(ColorSensor.class, "Left Color");
            rightColor = hardwareMap.get(ColorSensor.class, "Right Color");
            */
            }

    //Setters for Motor Power and Mode
    //speeds up runMode setting
    public void setRunMode(String mode) {
            String runMode = Convert.runMode(mode);
            switch (runMode){
            case "using":
            setRunMode("s");
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            break;
            case "position":
            setRunMode("s");
            leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            middleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            break;
            case "without":
            setRunMode("s");
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            break;
            case "stop and reset":
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            break;
    default:
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            break;
            }
            }
    public void setRobotPower(double power) {
            double sPower = power * .9;
            setForwardPower(sPower);
            strafe(sPower);
            }
    public void setForwardPower(double power) {
            double sPower = power * .9;
            rightMotor.setPower(sPower);
            leftMotor.setPower(sPower);
            }
    public void turn(double power, String directionInp) {
            String direction = Convert.direction(directionInp);
            power *= 0.9;
            switch (direction) {
            case "f":
            setForwardPower(power);
            break;
            case "b":
            setForwardPower(-power);
            break;
            case "l":
            leftMotor.setPower(power);
            rightMotor.setPower(-power);
            break;
            case "r":
            leftMotor.setPower(-power);
            rightMotor.setPower(power);
            break;
    default:
            break;
            }
            }
    public void turnRight(double power) {
            power *= 0.9;
            turn(power, "r");
            }
    public void turnLeft(double power) {
            power *= 0.9;
            turn(power, "l");
            }
    public void strafe(double power) {
            power *= 0.9;
            middleMotor.setPower(power);
            }
    public void move(double power, double angle){
            power *= 0.9;
            double xPower = power * Math.cos(angle);
            double yPower = power * Math.sin(angle);

            rightMotor.setPower(yPower);
            leftMotor.setPower(yPower);
            middleMotor.setPower(xPower);
            }
    public void moveDegrees(double power, double angle) {
            move(power, Math.toRadians(angle));
            }
    public void move(double power, double x, double y) {
            move(power, Convert.angle(x, y));
            }
    public void setRightPower(double power) {
            power *= 0.9;
            rightMotor.setPower(power);
            }
    public void setLeftPower(double power) {
            power *= 0.9;
            leftMotor.setPower(power);
            }
    public void setMiddlePower(double power) {
            power *= 0.9;
            middleMotor.setPower(power);
            }

    //Autonomous Movement
    public void moveAuton(double distance, double angle, double power, String unit) {
            power *= 0.9;
            //uses trig and physics to divide distance vector into x and y components
            double xDistance = distance * Math.cos(angle);
            double yDistance = distance * Math.sin(angle);

            //Calculates Number of Rotations Necessary
            //y is forward/backwards, x is left/right
            double yRotations = rightWheel.getNumOfRots(yDistance, unit);
            double xRotations = middleWheel.getNumOfRots(xDistance, unit);

            //Calculates ticks to get that movement
            double yTicks = yRotations * ticksPerRotationForward;
            double xTicks = xRotations * ticksPerRotationSideways;

            //rounds the ticks:
            int roundedYTicks = Convert.round(yTicks);
            int roundedXTicks = Convert.round(xTicks);

            //Sets it to run to position
            setRunMode("p");

            rightMotor.setTargetPosition(roundedYTicks);
            leftMotor.setTargetPosition(roundedYTicks);
            middleMotor.setTargetPosition(roundedXTicks);

            setRobotPower(power);
            position[0] += xDistance;
            position[1] += yDistance;
            }
    public void moveSideways(double power, double distance){
            power *= 0.9;
            setUpEncodersForDistance(distance);
            //set drive power
            middleMotor.setPower(power);
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            //Has motors run until position is reached
            while(middleMotor.isBusy()){
            //Waits
            }
            //updates position
            position[0] += distance;
            //Stops driving
            turnOff();
            //Changes mode back to normal
            setRunMode("u");
            }
    public void moveFoward(double power, double distance){
            power *= 0.9;
            setUpEncodersForDistance(distance);
            //set drive power
            middleMotor.setPower(0);
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            //Has motors run until position is reached
            while(leftMotor.isBusy()){
            //Waits
            }
            //Updates Position
            position[1] += distance;
            //Stops driving
            turnOff();
            setRunMode("u");
            }
    public void moveBackwards(double power, double distance){
            power *= 0.9;
            setUpEncodersForDistance(distance);
            //set drive power
            middleMotor.setPower(0);
            leftMotor.setPower(-power);
            rightMotor.setPower(-power);
            //Has motors run until position is reached
            while(leftMotor.isBusy()){
            //Waits
            }
            position[1] -= distance;
            //Stops driving
            turnOff();
            //Changes mode back to normal
            //Reset encoder values
            setRunMode("u");
            }
    public void setUpEncodersForDistance(double distance){
            //Switch to position mode encoder values
            setRunMode("p");
            //Set target position
            leftMotor.setTargetPosition(leftWheel.getNumOfTicks(distance));
            rightMotor.setTargetPosition(rightWheel.getNumOfTicks(distance));
            middleMotor.setTargetPosition(middleWheel.getNumOfTicks(distance));
            }
    public void turnRight(double power, double distance){
            power *= 0.9;
            setUpEncodersForDistance(distance);
            rightMotor.setPower(power);
            leftMotor.setPower(-power);
            middleMotor.setPower(0);
            }

    //Could use this instead of individual turning methods
    public void turnOff(){
            leftMotor.setPower(0);
            rightMotor.setPower(0);
            middleMotor.setPower(0);
            }
           }
