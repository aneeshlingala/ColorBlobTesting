package org.firstinspires.ftc.teamcode.util;

public class trigDistance {

    public static double calculateDistance(double opposite, double angle){
        double distance = ((opposite) / Math.tan(angle));
        return distance;
    }
}