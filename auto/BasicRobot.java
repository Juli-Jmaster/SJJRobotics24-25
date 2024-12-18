package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.teamcode.auto.servo.FixedPositionServo;

public interface BasicRobot {

    FixedPositionServo outtakeAngle = new FixedPositionServo("outtakeAngle", new double[]{0.44, 0.58});
    FixedPositionServo intakeAngle = new FixedPositionServo("intakeAngle", new double[]{0.73, 0.08});

}
