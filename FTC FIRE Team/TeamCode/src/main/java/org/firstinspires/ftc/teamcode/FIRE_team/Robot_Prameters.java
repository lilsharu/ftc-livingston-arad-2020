package org.firstinspires.ftc.teamcode.FIRE_team;

/**
 * Created by lior on 10/8/2019.
 */


/**
 * all the information here correct for start position
 */
public class Robot_Prameters {

    private double Robot_Radius;
    private Location centerOfTheRobot ;

    public Robot_Prameters(double robot_Radius, Location centerOfTheRobot) {
        Robot_Radius = robot_Radius;
        this.centerOfTheRobot = centerOfTheRobot;
    }

    public double getRobot_Radius() {
        return Robot_Radius;
    }

    public Location getCenterOfTheRobot() {
        return centerOfTheRobot;
    }
}
