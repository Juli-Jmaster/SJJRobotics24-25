package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

// a class for handling A DcMotor and it's simplified functions for it
public class Motor {
    public String motorname;
    private DcMotor motor;
    private boolean isReversed;
    private boolean autonomous;
    private boolean brake;

    public Motor(String name, boolean reversed, boolean brake, boolean autonomous){
        this.motorname = name;
        this.isReversed = reversed;
        this.autonomous = autonomous;
        this.brake = brake;
    }

    //setup for when init hit in auto
    public void setupMotor(){
        if(isReversed){motor.setDirection(DcMotor.Direction.REVERSE);}
        if(autonomous){setMode(DcMotor.RunMode.RUN_USING_ENCODER);}
        if(brake){motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);}
    }
    public void setMotor(DcMotor motor) {
        this.motor = motor;
    }
    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }

    //sets its target and tell robot to move
    public void move(int ticks) {
        int position = motor.getCurrentPosition() + ticks;
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
    //is true while the robot is moving to its target position
    public boolean isBusy(){
        return motor.isBusy();
    }
    public void stopMotor() {
        motor.setPower(0);
    //    motor.setTargetPosition(motor.getCurrentPosition());
    }

    public DcMotor getMotor() {
        return motor;
    }

}
