package org.firstinspires.ftc.teamcode.auto;

//where all functions for tolerance and if passed target
//basically a utils class
public class Utils {

//    public enum DIRECTION {LEFT, RIGHT, UNKOWN}

    // Function to check if an angle is within 5 degrees of the target angle
    public static boolean isWithinTolerance(int angle, int target, int tolerance) {
        int lowerBound = (target - tolerance) ;
        int upperBound = (target + tolerance) ;

        if(lowerBound <= upperBound){
            return lowerBound <= angle && angle <= upperBound;
        } else {
            return lowerBound >= angle && angle >= upperBound;

        }
    }
    public static boolean isWithinTolerance(double angle, double target, double tolerance) {
        double lowerBound = (target - tolerance);
        double upperBound = (target + tolerance);

        if (lowerBound <= upperBound) {
            return lowerBound <= angle && angle <= upperBound;
        } else {
            return lowerBound >= angle && angle >= upperBound;

        }
    }

    public static boolean passedTarget(int input, int target){
        if(target > 0){
            return input >= target;
        } else {
            return input <= target;
        }
    }
}