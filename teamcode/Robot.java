/*
 * The Specific Robot Class for FIRE
 * Created by Shourya Bansal and Ally Mintz
 */

package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.Auton.Wheel;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;



public class Robot {
    //Creates DcMotors
    private DcMotor rightFrontMotor;
    private DcMotor leftFrontMotor;
    private DcMotor middle_drive;
    //private DcMotor elevatorMotor;

    //Creates Corresponding Wheels
    private Wheel rightWheel;
    private Wheel leftWheel;
    private Wheel middleWheel;

    //Creates Sensor to Direct
    private ColorSensor colorSensor;
    //private ColorSensor rightColor;
/*
    //Creates Servos which are being used
    private Servo clawServo;
    private Servo rightPlateServo;
    private Servo leftServo;
*/
    //Creates an array to store the robot's position after every movement
    double[] position = new double[2];

    //Creates a timer to calculate elapsed time
    ElapsedTime runtime = new ElapsedTime();

    //Variables and values which we need to know
    double ticksPerRotation;

    //Constructors to create the Robot
    public Robot(DcMotor rightFrontMotor, DcMotor leftFrontMotor, DcMotor middle_drive, Wheel rightWheel, Wheel leftWheel, Wheel middleWheel, ColorSensor colorSensor, double ticksPerRotation)
    {
        this.rightFrontMotor = rightFrontMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.middle_drive= middle_drive;
        //this.elevatorMotor = elevator;
        this.rightWheel = rightWheel;
        this.leftWheel = leftWheel;
        this.middleWheel = middleWheel;
        //this.leftColor = leftColor;
        this.colorSensor = colorSensor;
        /*
        this.clawServo = clawServo;
        this.rightPlateServo = rightPlateServo;
        this.leftServo = leftPlateServo;
        */
        this.ticksPerRotation = ticksPerRotation;
    }
    public Robot(DcMotor rightFrontMotor, DcMotor leftFrontMotor, DcMotor middle_drive, Wheel rightWheel, Wheel leftWheel, Wheel middleWheel, double ticksPerRotation) {
        this.rightFrontMotor = rightFrontMotor;
        this.leftFrontMotor = leftFrontMotor;
        this.middle_drive = middle_drive;
        this.rightWheel = rightWheel;
        this.leftWheel = leftWheel;
        this.middleWheel = middleWheel;
        this.ticksPerRotation = ticksPerRotation;
    }

    //Initializes Robot
    HardwareMap hardwareMap;
    public void init(HardwareMap HardwareMap) {
        hardwareMap = HardwareMap;
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
        middle_drive = hardwareMap.get(DcMotor.class, "middle_drive");
        //elevatorMotor = hardwareMap.get(DcMotor.class, "Claw Motor");

        //this is guess and test, change if necessary
        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        middle_drive.setDirection(DcMotor.Direction.FORWARD);
        //elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    //Setters for Motor Power
    public void setMotorPower(DcMotor motor, double power) {
        motor.setPower(power);
    }
    public void setForwardPower(double power) {
        rightFrontMotor.setPower(power);
        leftFrontMotor.setPower(power);
    }
    public void turn(double power, String directionInp) {
        String direction = direction(directionInp);
        switch (direction) {
            case "f":
                setForwardPower(power);
                break;
            case "b":
                setForwardPower(-power);
                break;
            case "l":
                leftFrontMotor.setPower(power);
                rightFrontMotor.setPower(-power);
            case "r":
                leftFrontMotor.setPower(-power);
                rightFrontMotor.setPower(power);
            default:
                break;
        }
    }
    public void turnRight(double power) {
        turn(power, "r");
    }
    public void turnLeft(double power) {
        turn(power, "l");
    }
    public void strafe(double power) {
        middle_drive.setPower(power);
    }
    public void move(double power, double angle){
        double xPower = power * Math.cos(angle);
        double yPower = power * Math.sin(angle);

        rightFrontMotor.setPower(yPower);
        leftFrontMotor.setPower(yPower);
        middle_drive.setPower(xPower);
    }
    public void moveDegrees(double power, double angle) {
        move(power, Math.toRadians(angle));
    }

    //Autonomous Movement
    public void moveAuton(double distance, double angle, String unit) {
        //uses trig and physics to divide distance vector into x and y components
        double xDistance = distance * Math.cos(angle);
        double yDistance = distance * Math.sin(angle);

        //Calculates Number of Rotations Necessary
        double yRotations = middleWheel.getNumOfRots(xDistance, unit);
        double xRotations = rightWheel.getNumOfRots(yDistance, unit);

        //Calculates ticks to get that movement
        double yTicks = yRotations * ticksPerRotation;
        double xTicks = xDistance * ticksPerRotation;

        //rounds the ticks:
        int roundedYTicks = round(yTicks);
        int roundedXTicks = round(xTicks);

        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontMotor.setTargetPosition(roundedYTicks);
        leftFrontMotor.setTargetPosition(roundedYTicks);
        middle_drive.setTargetPosition(roundedXTicks);
    }

    public void moveSideways(double power, int distance){
        SetUpEncodersForDistance(distance);
        //set drive power
        middle_drive.setPower(power);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        //Has motors run until position is reached
        while(middle_drive.isBusy()){
            //Waits
        }

        //Stops driving
        turnOff();
        //Changes mode back to normal
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        middle_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveFoward(double power, int distance){
        SetUpEncodersForDistance(distance);
        //set drive power
        middle_drive.setPower(0);
        leftFrontMotor.setPower(power);
        rightFrontMotor.setPower((-1*power));
        //Has motors run until position is reached
        while(leftFrontMotor.isBusy()){
            //Waits
        }

        //Stops driving
        turnOff();
        //Changes mode back to normal
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        middle_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveBackwards(double power, int distance){
        SetUpEncodersForDistance(distance);
        //set drive power
        middle_drive.setPower(0);
        leftFrontMotor.setPower((-1*power));
        rightFrontMotor.setPower(power);
        //Has motors run until position is reached
        while(leftFrontMotor.isBusy()){
            //Waits
        }

        //Stops driving
        turnOff();
        //Changes mode back to normal
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        middle_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetUpEncodersForDistance(int distance){
        //Reset encoder values
        leftFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        rightFrontMotor.setMode(DcMotor.RunMode.RESET_ENCODERS);
        middle_drive.setMode(DcMotor.RunMode.RESET_ENCODERS);
        //Set target position
        leftFrontMotor.setTargetPosition(distance);
        rightFrontMotor.setTargetPosition(distance);
        middle_drive.setTargetPosition(distance);
        //Switch to position mode
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        middle_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void moveRight(double power){
        rightFrontMotor.setPower(power);
        leftFrontMotor.setPower(0);
        middle_drive.setPower(0);
    }
    //Could use this instead of individual turning methods
    public void turnOff(){
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        middle_drive.setPower(0);
    }



    //Deciphering methods and other static methods:
    public String direction(String directionInput) {
        if (directionInput.equalsIgnoreCase("right") || directionInput.equalsIgnoreCase("r")){
            return "r";
        }
        else if (directionInput.equalsIgnoreCase("left") || directionInput.equalsIgnoreCase("l")) {
            return "l";
        }
        else if (directionInput.equalsIgnoreCase("back") || directionInput.equalsIgnoreCase("backwards")||directionInput.equalsIgnoreCase("b")) {
            return "b";
        }
        else if (directionInput.equalsIgnoreCase("forward") || directionInput.equalsIgnoreCase("front") || directionInput.equalsIgnoreCase("f")) {
            return "f";
        }
        else {
            throw new Error ("Your direction couldn't be found. Please try a different direction which is already programmed or add a new case");
        }
    }
    public static int round(double input) {
        return (int)Math.round(input);
    }
    public static double round(double input, int places) {
        double newNum = input * Math.pow(10, places);
        newNum += 0.5;
        newNum = (int)newNum;
        newNum /= Math.pow(10, places);
        return newNum;
    }

    //Other methods
    public static String usedUnit(String unit) {
        //Sees if the user put in inches
        if (unit.equals("\"") || unit.equals("inches") || unit.equals("Inches") || unit.equals("in") || unit.equals("in.")) {
            return "in";
        }
        //Sees if the user put in feet
        else if (unit.equals("'") || unit.equals("feet") || unit.equals("Feet") || unit.equals("ft") || unit.equals("ft.")) {
            return "ft";
        }
        //Sees if the user put in millimeters
        else if (unit.equals("mm") || unit.equals("MM") || unit.equals("millimeters") || unit.equals("millimetres") || unit.equals("milimeters") || unit.equals("milli")){
            return "mm";
        }
        //If it is none of the above
        else {
            throw new Error("The Unit Chosen Doesn't Match Any of the Options. You said \""
                    + unit +
                    "\" but that was not an option");
        }
    }
    public static double convertToInches(double currentValue, String currentUnit) {
        String unit = usedUnit(currentUnit);
        //tests each case
        switch(unit) {
            case "in":
                return currentValue;
            case "ft":
                return currentValue / 12.0;
            case "mm":
                return currentValue / 25.4;
            default:
                throw new Error(
                        "The Unit Chosen Doesn't Match Any of the Options. You said \""
                                + unit +
                                "\" but that was not an option");
        }
    }
    public static double convertToMillimeters(double currentValue, String currentUnit){
        //Sends this to the convertToInches method and converts it to inches, then converts back to millimeters
        return 25.4 * convertToInches(currentValue, usedUnit(currentUnit));
    }

}
