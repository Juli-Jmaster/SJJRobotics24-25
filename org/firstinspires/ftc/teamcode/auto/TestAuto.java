package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TestAuto  extends LinearOpMode implements DriveMainAuto, BasicRobot{

    @Override
    public void runOpMode() throws InterruptedException {
        movementStraight(3, 1, 0, telemetry);
    }
}
