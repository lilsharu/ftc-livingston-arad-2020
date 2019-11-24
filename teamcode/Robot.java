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
        private DcMotor elevator;

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

    //Constructors to create the Robot
        public Robot(DcMotor rightMotor, DcMotor leftMotor, DcMotor middleMotor, DcMotor elevator, Wheel rightWheel, Wheel leftWheel, Wheel middleWheel, ColorSensor leftColor, ColorSensor rightColor, Servo clawServo, Servo rightPlateServo, Servo leftPlateServo) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.middleMotor = middleMotor;
            this.elevator = elevator;
            this.rightWheel = rightWheel;
            this.leftWheel = leftWheel;
            this.middleWheel = middleWheel;
            this.leftColor = leftColor;
            this.rightColor = rightColor;
            this.clawServo = clawServo;
            this.rightPlateServo = rightPlateServo;
            this.leftPlateServo = leftPlateServo;
        }
        public Robot(DcMotor rightMotor, DcMotor leftMotor, DcMotor middleMotor) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.middleMotor = middleMotor;
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

    //Deciphering methods:
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
}
