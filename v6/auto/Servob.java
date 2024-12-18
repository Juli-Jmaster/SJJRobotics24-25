package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Servo;

public class Servob
{
    public final String servoName;
    private Servo servo;
    private final double[] positions;
    public Servob(String servoName, double[] positions){
        this.servoName = servoName;
        this.positions = positions;
    }

    public void setupServo(){

    }
    public void setServo(Servo servo){
        this.servo = servo;
    }

/*    public void open(){
        servo.setPosition(openPosition);
    }
    public void close(){
        servo.setPosition(closePosition);
    }


    public void increase(){
        servo.setPosition(servo.getPosition()+increaseAmount);
    }
    public void decrease(){
        servo.setPosition(servo.getPosition()-increaseAmount);
    }*/
    public void set(int num){
        servo.setPosition(positions[num]);
    }
    public double get(int num){
        return positions[num];
    }
    public Servo getServo(){
        return servo;
    }
    public double getPos(){
        return servo.getPosition();
    }
}
