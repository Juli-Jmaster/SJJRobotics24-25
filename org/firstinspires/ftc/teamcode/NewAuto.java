package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.BasicRobot;
import org.firstinspires.ftc.teamcode.auto.DriveMainAuto;
import org.firstinspires.ftc.teamcode.auto.UpdatePowerTypes;

import static org.firstinspires.ftc.teamcode.auto.CheckDriveStraight.passedTarget;

@Autonomous
public class NewAuto extends LinearOpMode implements DriveMainAuto, BasicRobot {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        //load motors things
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        loadAll(hardwareMap);
        outtakeAngle.set(TRANSFER);
        outtakeClaw.set(CLOSE);
        intakeAngle.set(TRANSFER);
        intakeClaw.set(OPEN);
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //send telemetry data and wait for start
        DcMotorEx sideways1 = hardwareMap.get(DcMotorEx.class, "sideways");


        telemetry.update();
        waitForStart();
        runtime.reset();

        boolean forward = true;
        boolean where = false;
        double pos;
        int straightFacing = 180;

        movementStraight(20,1, 180, telemetry);
    }

    private void waitMe(double sec){
        runtime.reset();
        while (runtime.seconds() < sec) {
        }
    }
}
