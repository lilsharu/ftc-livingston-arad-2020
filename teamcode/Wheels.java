package org.firstinspires.ftc.teamcode;

public class Wheels {
    private double radius;
    private String type;
    private String brand;

    public Wheels(double radius, String type, String brand) {
        this.radius = radius;
        this.type = type;
        this.brand = brand;
    }

    public Wheels(double radius, String type) {
        this(radius, type, null);
    }

    public Wheels(double radius) {
        this(radius, "Tank", null);
    }

    public double getRadius() {
        return radius;
    }

    public void setRadius(double radius) {
        this.radius = radius;
    }

    public String getType() {
        return type;
    }

    public void setType(String type) {
        this.type = type;
    }

    public String getBrand() {
        return brand;
    }

    public void setBrand(String brand) {
        this.brand = brand;
    }

    @Override
    public String toString() {
        return type + " wheel from " + brand + " with a radius of " + radius + "millimeters";
    }

    public double circumference() {
        return rotationDistanceMilli();
    }

    public double rotationDistanceMilli() {
        return 2 * Math.PI * radius;
    }

    public double rotationDistanceInches() {
        return rotationDistanceMilli() / 25.4;
    }

    public double circumferenceInches() {
        return rotationDistanceInches();
    }

    public double rotationDistanceFeet() {
        return rotationDistanceInches() / 12;
    }

    public double circumferenceFeet() {
        return rotationDistanceFeet();
    }

    public int[] ticksForDistanceRadians(double ticksPerRotation, double distance, double angle, String unit) {
        int[] array = new int[2];
        double x;
        double y;
        x = distance * Math.cos(angle);
        y = distance * Math.sin(angle);

        if (unit.equals("\"") || unit.equals("inches") || unit.equals("Inches") || unit.equals("in") || unit.equals("in.")) {
            array[0] = (int)Math.round((x / circumferenceInches()) * ticksPerRotation);
            array[1] = (int) Math.round((y / circumferenceInches()) * ticksPerRotation);
            return array;
        }
        else if (unit.equals("'") || unit.equals("feet") || unit.equals("Feet") || unit.equals("ft") || unit.equals("ft.")) {
            array[0] = (int)Math.round((x / circumferenceFeet()) * ticksPerRotation);
            array[1] = (int)Math.round((y / circumferenceFeet()) * ticksPerRotation);
            return array;
        }
        else if (unit.equals("mm") || unit.equals("MM") || unit.equals("millimeters") || unit.equals("millimetres") || unit.equals("milimeters")) {
            array[0] = (int) Math.round((x / circumference()) * ticksPerRotation);
            array[1] = (int) Math.round((y / circumference()) * ticksPerRotation);
            return array;
        }
        else return null;
    }

}
