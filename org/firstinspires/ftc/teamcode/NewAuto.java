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

        elavator1.setMotor(hardwareMap.get(DcMotor.class, elavator1.motorname));
        elavator1.setupMotor();
        elavator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elavator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elavator2.setMotor(hardwareMap.get(DcMotor.class, elavator2.motorname));
        elavator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elavator2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //send telemetry data and wait for start
        DcMotorEx sideways1 = hardwareMap.get(DcMotorEx.class, "sideways");


        telemetry.update();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("cur", elavator1.getMotor().getCurrentPosition());
            telemetry.update();
            // elavator2.setPower(0.9);ac
            // elavator1.setPower(0.9);
        }

    }
}
