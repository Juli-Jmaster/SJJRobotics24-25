package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.auto.Utils.isWithinTolerance;

// class for handling the IMU and its calculations
//because of how the robot gets its data in -180 to 180
//the defalt straight is 180 becasue we mad it so now the values are between 0 and 360
//just added +180 to original number
//thats why the new straight ahead is 180
public class InterfaceErrorIMU {
    private final String name;
    private double rotationLeft = 0;
    private double power = 0.1;
    private IMU imu;

    public InterfaceErrorIMU(String name){
        this.name = name;
    }


    public String getName() {
        return name;
    }
    public void setImu(IMU imu){
        this.imu = imu;
    }
    public IMU getImu() {
        return imu;
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    // so now 180 is straight ahead and directly behind is 0 or 360
    public double getYaw(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;
    }
    //get if it not facing the target with a tolerance within a time limit
    //broken timer
    public boolean notFacingTimer(int target, ElapsedTime runtime, int time){
        rotationLeft=0;
        boolean correctDriction=false;
        int cur = (int)getYaw();
        if (!isWithinTolerance(cur, target, 3)){
            rotationLeft+=power;
            correctDriction=true;
        }
        if (!isWithinTolerance(cur, target, 7)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 10)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 15)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 20)){
            rotationLeft+=power;
        }
        // if(runtime.seconds() > time){
        //     correctDriction=true;
        // }
        return correctDriction;
    }

    //get if it not facing the target with a tolerance
    public boolean notFacing(int target){
        rotationLeft=0;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        double angleDifference = currentYaw - target;

        if (angleDifference > 180) {
            angleDifference -= 360;
        }

        if (angleDifference < -180) {
            angleDifference += 360;
        }

        if (angleDifference > 0) {
            rotationLeft = MovementCurves.quadraticCurve(angleDifference/360);
        }

        if (angleDifference < 0) {
            rotationLeft = -MovementCurves.quadraticCurve(-angleDifference/360);
        }
        if (!isWithinTolerance((int) currentYaw, target, 3)){
            return true;
        }else {return false;}

    }
    //returns the power to the left side wheel to turn to correct
    public double getRotationLeftPower(int target) {
        return rotationLeft;//*(turnToCorrectSide(getYaw(), target) ? 1 : -1);
    }

    public void face(double direction) {

        //forward 0
        //right -90, 270
        //backwards 180, -180
        //left 90

        double currentYaw;
        double angleDifference = 180;
        double rX = 1;

        if (direction > 180) {
            direction -= 360;
        }
        currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        angleDifference = currentYaw - direction;

        if (angleDifference > 180) {
            angleDifference -= 360;
        }

        if (angleDifference < -180) {
            angleDifference += 360;
        }

        if (angleDifference > 0) {
            rX = MovementCurves.quadraticCurve(angleDifference / 360);
        }

        if (angleDifference < 0) {
            rX = -MovementCurves.quadraticCurve(-angleDifference / 360);
        }
        if (rX > 0 && rX < .1) {
            rX = .1;
        }
        if (rX < 0 && rX > -.1) {
            rX = -.1;
        }
    }
    //done

    public boolean gotToTarget(int target) {
        return Utils.isWithinTolerance(getYaw(), target, 0.15);
    }
}
