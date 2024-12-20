package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.auto.servo.AjustableServo;
import org.firstinspires.ftc.teamcode.auto.servo.FixedPositionServo;

public interface BasicRobot {

    // first tvalue is the transfer postion and other is outside position
    FixedPositionServo outtakeAngle = new FixedPositionServo("outtakeAngle", new double[]{0.4375, 0.58});
    FixedPositionServo intakeAngle = new FixedPositionServo("intakeAngle", new double[]{0.73, 0.08});

    //first position is out and second is all the way in
    AjustableServo intakeSlide1 = new AjustableServo("intakeSlide1", 0,0,0.35,0.0025);
    AjustableServo intakeSlide2 = new AjustableServo("intakeSlide2", 1, 0.65, 1, 0.0025);
}

