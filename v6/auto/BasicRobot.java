package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.Servo;

public interface BasicRobot {

    Servob outtakeAngle = new Servob("outtakeAngle", new double[]{0.44, 0.58});
    Servob intakeAngle = new Servob("intakeAngle", new double[]{0.73, 0.08});

}
