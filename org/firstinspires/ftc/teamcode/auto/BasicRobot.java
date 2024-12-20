package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.auto.servo.AjustableServo;
import org.firstinspires.ftc.teamcode.auto.servo.FixedPositionServo;

public interface BasicRobot {

    // first value is the transfer position and other is outside position
    int TRANSFERINTAKE = 0;
    int TRANSFEROUTTAKE = 0;
    int RETRIEV = 1;
    int PLACEOUTTAKE = 1;
    FixedPositionServo outtakeAngle = new FixedPositionServo("outtakeAngle", new double[]{0.4375, 0.58});
    FixedPositionServo intakeAngle = new FixedPositionServo("intakeAngle", new double[]{0.73, 0.08});

    //first position is out and second is all the way in
    int SLIDEOUT = 0;
    int SLIDEIN = 1;
    AjustableServo intakeSlide1 = new AjustableServo("intakeSlide1", 0,0,0.35,0.0025);
    AjustableServo intakeSlide2 = new AjustableServo("intakeSlide2", 1, 0.65, 1, 0.0025);

    int OPEN = 0;
    int CLOSE = 1;
    FixedPositionServo outtakeClaw = new FixedPositionServo("outtakeClaw", new double[]{0.036, 0.3});
    FixedPositionServo intakeClaw = new FixedPositionServo("intakeClaw", new double[]{0.0D, 0.05});
}

