package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.auto.servo.AjustableServo;
import org.firstinspires.ftc.teamcode.auto.servo.FixedPositionServo;
//where all the extra motors and servos are set
public interface BasicRobot {

    // first value is the transfer position and other is outside position
    int TRANSFER = 0;
    int GRAB = 1;
    int PLACE = 1;
    FixedPositionServo outtakeAngle = new FixedPositionServo("outtakeAngle", new double[]{0.441+.033, 0.589+.03});
    FixedPositionServo intakeAngle = new FixedPositionServo("intakeAngle", new double[]{0.75, 0.03});

    //first position is out and second is all the way in
    int SLIDEOUT = 0;
    int SLIDEIN = 1;
    AjustableServo intakeSlide1 = new AjustableServo("intakeSlide1", 0,0,0.35,0.0025);
    AjustableServo intakeSlide2 = new AjustableServo("intakeSlide2", 1, 0.65, 1, 0.0025);

    int OPEN = 0;
    int CLOSE = 1;
    int SLIGHTCLOSE = 2;
    FixedPositionServo outtakeClaw = new FixedPositionServo("outtakeClaw", new double[]{0.2, 0.34});
    FixedPositionServo intakeClaw = new FixedPositionServo("intakeClaw", new double[]{0.0D, 0.1, 0.7});
    int CEN= 0 ;
    int LEFT = 1;
    int RIGHT = 2;
    AjustableServo intakeRotate = new AjustableServo("intakeRotate", 0.49, 0.427, 0.553, 0.00025);


    //need to fix now that motor have brake enabled; all needs to done in motor class thou
    Motor elavator1 = new Motor("elavator1", true, true,false);
    Motor elavator2 = new Motor("elavator2", false, true,false);

    int HIGH_BASKET = 3300;
    //loads the extra motors and servos for this season
    default void loadAll(HardwareMap hardwareMap){
        //motors
        elavator1.setMotor(hardwareMap.get(DcMotor.class, elavator1.motorname));
        elavator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elavator1.setupMotor();
        elavator2.setMotor(hardwareMap.get(DcMotor.class, elavator2.motorname));
        elavator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elavator2.setupMotor();

        //servos
        outtakeAngle.setServo(hardwareMap.get(Servo.class, outtakeAngle.servoName));
        intakeAngle.setServo(hardwareMap.get(Servo.class, intakeAngle.servoName));
        intakeSlide1.setServo(hardwareMap.get(Servo.class, intakeSlide1.servoName));
        intakeSlide2.setServo(hardwareMap.get(Servo.class, intakeSlide2.servoName));
        outtakeClaw.setServo(hardwareMap.get(Servo.class, outtakeClaw.servoName));
        intakeClaw.setServo(hardwareMap.get(Servo.class, intakeClaw.servoName));
        intakeRotate.setServo(hardwareMap.get(Servo.class, intakeRotate.servoName));

    }

    default void elevator(int ticks){
        elavator1.moveNoPower(ticks);
        elavator2.moveNoPower(ticks);
        while(elavator1.getMotor().isBusy()){
            elavator2.setPower(0.7);
            elavator1.setPower(0.7);
        }
        elavator2.setPower(0.1);
        elavator1.setPower(0.1);
    }
    default void elevatorWhileMove(){
        if (elavator1.getMotor().isBusy()){
            elavator2.setPower(0.7);
            elavator1.setPower(0.7);
        }
        if(!(elavator1.getMotor().isBusy())){
            elavator2.setPower(0.1);
            elavator1.setPower(0.1);
        }
    }

    default void elevatorMove(int ticks){
        elavator1.move(ticks);
        elavator2.move(ticks);
    }

    default void elevatorWait(){
        while (elavator1.getMotor().isBusy()){

        }
    }
}


