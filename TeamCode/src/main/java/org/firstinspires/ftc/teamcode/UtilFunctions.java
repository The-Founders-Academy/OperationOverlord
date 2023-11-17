package org.firstinspires.ftc.teamcode;

public class UtilFunctions {
    public static double clamp(double valueToClamp, double lowerBound, double upperBound) {
        if(valueToClamp < lowerBound) {
            return lowerBound;
        } else if(valueToClamp > upperBound) {
            return upperBound;
        } else {
            return valueToClamp;
        }
    }
}
