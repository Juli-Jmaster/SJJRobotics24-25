package org.firstinspires.ftc.teamcode.auto.servo;

public class ClawServo extends Servob{
    public  double openPos;
    public  double closePos;

    public ClawServo(String servoName, double openPos, double closePos) {
        super(servoName);
        this.closePos = closePos;
        this.openPos = openPos;
    }

    public void openClaw(){
        servo.setPosition(openPos);
    }

    public void closeClaw(){
        servo.setPosition(closePos);
    }
}
