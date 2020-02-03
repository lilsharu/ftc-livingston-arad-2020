/*
 * The Wheel Class implemented in the FIRE Team Code
 * Designed by Shourya Bansal and Ally Mintz
 */
 
package org.firstinspires.ftc.teamcode.teamcode;
public class Wheel {
    private double radius; //This is in millimeters
    private String position; //The Robot Position (eg Front Left, Middle, etc)
    private String type; //The type of wheel (that is, omni, mecanum, tank, or any other type)
    private String orgUn;
    private int ticks;

    //Constructors for a Wheel Object
    public Wheel(double radius, int ticks, String position, String type, String unit) {
        //Sets instance variables equal to User Arguments
        this.radius = Convert.toInches(radius, unit);
        this.ticks = ticks;
        this.position = position;
        this.type = type;
        this.orgUn = unit;
    }

    public Wheel(double radius, int ticks, String type, String unit) {
        //Sets instance variables equal to User arguments and leaves position unassigned
        this(radius, ticks, "Unassigned", type, unit);
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

    public double getNumOfRots(double distance, String unit) {
        //converts distance to inches
        double inchDist = Convert.toInches(distance, unit);
        return distance / getDPR();
    }

    public double getDPR() {
        return radius * 2 * Math.PI;
    }

    public double getNumOfRot(double distance) {
        return distance / getDPR();
    }

    public int getNumOfTicks(double dist) {
        double nor = getNumOfRot(dist);
        return (int) Math.round(nor * ticks);
    }

    //What will print when we convert it to a String:
    @Override
    public String toString() {
        return "Radius: " + radius + ", Position on Robot: " + position + ", Type of Wheel: " + type;
    }
}

