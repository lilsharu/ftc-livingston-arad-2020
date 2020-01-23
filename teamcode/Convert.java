package org.firstinspires.ftc.teamcode;

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
    public static String direction(String directionInput) {
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
    public static String runMode(String mode) {
        if (mode.equalsIgnoreCase("using") || mode.equalsIgnoreCase("u") || mode.equalsIgnoreCase("with") || mode.equalsIgnoreCase("encoders"))
            return "using";
        else if (mode.equalsIgnoreCase("without") || mode.equalsIgnoreCase("w") || mode.equalsIgnoreCase("no") || mode.equalsIgnoreCase("wout"))
            return "without";
        else if (mode.equalsIgnoreCase("position") || mode.equalsIgnoreCase("p") || mode.equalsIgnoreCase("to") || mode.equalsIgnoreCase("auton") || mode.equalsIgnoreCase("pos"))
            return "position";
        else if (mode.equalsIgnoreCase("reset") || mode.equalsIgnoreCase("stop") || mode.equalsIgnoreCase("stop and reset") || mode.equalsIgnoreCase("stop n reset") || mode.equalsIgnoreCase("s") || mode.equalsIgnoreCase("r"))
            return "stop and reset";
        else throw new Error("Your input, " + mode + ", was not an option. Either add it to the code or try another input");
    }
    public static double toInches(double currentValue, String currentUnit) {
        String unit = usedUnit(currentUnit);
        //tests each
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
    public static double toMillimeters(double currentValue, String currentUnit){
        //Sends this to the toInches method and converts it to inches, then converts back to millimeters
        return 25.4 * toInches(currentValue, usedUnit(currentUnit));
    }
    public static double toInches(int currentValue, String currentUnit) {
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
    public static double toMillimeters(int currentValue, String currentUnit){
        //Sends this to the toInches method and converts it to inches, then converts back to millimeters
        return 25.4 * toInches(currentValue, usedUnit(currentUnit));
    }
    public static int round(double input) {
        return (int)Math.round(input);
    }
    public static double round(double input, int places) {
        double num = input * Math.pow(10, places) + 0.5;
        int numRounded = (int) num;
        num = numRounded / Math.pow(10, places);
        return num;
    }
    public static double angle(double x, double y) {
        double angle = 0;
        try {
            angle = Math.atan(y/x);
        }
        catch (Exception e) {
            if (x == 0) {
                if (y > 0) {
                    angle = Math.PI / 2;
                }
                else if (y < 0) {
                    angle = 3 * Math.PI / 2;
                }
            }
            else if (y == 0) {
                if (x > 0) {
                    angle = 0.0;
                }
                else if (x < 0) {
                    angle = Math.PI;
                }
            }
            else angle = angle(x, y);
        }
        return angle;
    }
}
