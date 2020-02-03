package org.firstinspires.ftc.teamcode.FIRE_team.functions;

import com.qualcomm.robotcore.util.Range;

public interface driveProportional {
    /**
     * the interface fo the thread
     * @param distance
     * @param slow
     * @param vmax

     */
    double driveProportionalFunction(double distance, double slow, double vmax);
}
