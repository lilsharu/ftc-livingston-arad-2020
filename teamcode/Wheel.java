/*
 * The Wheel Class implemented in the FIRE Team Code
 * Designed by Shourya Bansal and Ally Mintz
 */
package org.firstinspires.ftc.teamcode;

public class Wheel {
    private double radius; //This is in millimeters
    private String position; //The Robot Position (eg Front Left, Middle, etc)
    private String type; //The type of wheel (that is, omni, mecanum, tank, or any other type)

    //Constructors for a Wheel Object
        public Wheel(double radius, String position, String type) {
            //Sets instance variables equal to User Arguments
            this.radius = radius;
            this.position = position;
            this.type = type;
        }
        public Wheel(double radius, String type) {
            //Sets instance variables equal to User arguments and leaves position unassigned
            this(radius, "Unassigned", type);
        }
        public Wheel(double radius) {
            //Sets radius and automatically sets type of wheel to Tank
            this(radius, "Unassigned", "Tank");
        }

    //Getters and Setters for Instance Variables to get values and to change values
        public double getRadius() {
            return radius;
        }
        public void setRadius(double radius) {
            this.radius = radius;
        }
        public String getPosition() {
            return position;
        }
        public void setPosition(String position) {
            this.position = position;
        }
        public String getType() {
            return type;
        }
        public void setType(String type) {
            this.type = type;
        }

    //Other methods
        public String usedUnit(String unit) {
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
        public double convertToInches(double currentValue, String currentUnit) {
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
        public double convertToMillimeters(double currentValue, String currentUnit){
            //Sends this to the convertToInches method and converts it to inches, then converts back to millimeters
            return 25.4 * convertToInches(currentValue, usedUnit(currentUnit));
        }
        public double getCircumference() {
            return radius * 2 * Math.PI;
        }
        public double getCircumferenceInches() {
            return convertToInches(getCircumference(), "mm");
        }
        public double getDPR(){
            return getCircumference();
        }
        public double getNumOfRots(double distance, String unit){
            //converts distance to inches
            double inchDist = convertToInches(distance, unit);
            return distance/getCircumference();
        }

    //What will print when we convert it to a String:
        @Override
        public String toString(){
             return "Radius: " + radius + ", Position on Robot: " + position + ", Type of Wheel: " + type;
        }

}
