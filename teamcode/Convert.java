package org.firstinspires.ftc.teamcode.FIRE;

public class Convert {
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
