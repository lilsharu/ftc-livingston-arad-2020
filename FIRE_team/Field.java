package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;

/**
 * Created by lior on 10/8/2019.
 */

public class Field {

    private double width;
    private double length;


    public Field(double width, double length) {
        this.width = width;
        this.length = length;
    }

    /**
     * checks if robot now in legal X position , its a basic version , we will change it
     * after same testings.
     * @param robot
     * @param currentLocation is current position of the center of the robot on the field
     * @return
     */
    public boolean IsMovementLegal_X_Axis(Robot_Prameters robot , Location currentLocation){

        double min  = currentLocation.getX_axis() - robot.getRobot_Radius() ;
        double max  = currentLocation.getX_axis() + robot.getRobot_Radius() ;
        if(min > 0 && max <length) return false;
        return true ;
    }
    /**
     * checks if robot now in legal Y position , its a basic version , we will change it
     * after same testings.
     * @param robot
     * @param currentLocation is current position of the center of the robot on the field
     * @return
     */
    public boolean IsMovementLegal_Y_Axis(Robot_Prameters robot , Location currentLocation){

        double min  = currentLocation.getY_axis() - robot.getRobot_Radius() ;
        double max  = currentLocation.getY_axis() + robot.getRobot_Radius() ;
        if(min > 0 && max <width) return false;
        return true ;

    }
}
