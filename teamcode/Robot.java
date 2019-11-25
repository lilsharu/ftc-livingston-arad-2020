/*
 * The Specific Robot Class for FIRE
 * Created by Shourya Bansal
 */
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot {
    //Creates DcMotors
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
        private Servo rightPlateServo;
        private Servo leftPlateServo;

    //Creates an array to store the robot's position after every movement
        double[] position = new double[2];

    //Creates a timer to calculate elapsed time
        ElapsedTime runtime = new ElapsedTime();

    //Variables and values which we need to know
        double ticksPerRotation;

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
            this.rightPlateServo = rightPlateServo;
            this.leftPlateServo = leftPlateServo;
            this.ticksPerRotation = ticksPerRotation;
        }
        public Robot(DcMotor rightMotor, DcMotor leftMotor, DcMotor middleMotor, Wheel rightWheel, Wheel leftWheel, Wheel middleWheel, double ticksPerRotation) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.middleMotor = middleMotor;
            this.rightWheel = rightWheel;
            this.leftWheel = leftWheel;
            this.middleWheel = middleWheel;
            this.ticksPerRotation = ticksPerRotation;
        }

    //Initializes Robot
        HardwareMap hardwareMap;
        public void init(HardwareMap aHardwareMap) {
            hardwareMap = aHardwareMap;
            leftMotor = hardwareMap.get(DcMotor.class, "Left Motor");
            rightMotor = hardwareMap.get(DcMotor.class, "Right Motor");
            middleMotor = hardwareMap.get(DcMotor.class, "Middle Motor");
            //elevatorMotor = hardwareMap.get(DcMotor.class, "Claw Motor");

            //this is guess and test, change if necessary
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            middleMotor.setDirection(DcMotor.Direction.REVERSE);
            //elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        }

    //Setters for Motor Power
        public void setMotorPower(DcMotor motor, double power) {
            motor.setPower(power);
        }
        public void setForwardPower(double power) {
            rightMotor.setPower(power);
            leftMotor.setPower(power);
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
                    leftMotor.setPower(power);
                    rightMotor.setPower(-power);
                case "r":
                    leftMotor.setPower(-power);
                    rightMotor.setPower(power);
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
            middleMotor.setPower(power);
        }
        public void move(double power, double angle){
            double xPower = power * Math.cos(angle);
            double yPower = power * Math.sin(angle);

            rightMotor.setPower(yPower);
            leftMotor.setPower(yPower);
            middleMotor.setPower(xPower);
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

            rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            middleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            rightMotor.setTargetPosition(roundedYTicks);
            leftMotor.setTargetPosition(roundedYTicks);
            middleMotor.setTargetPosition(roundedXTicks);
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
}
