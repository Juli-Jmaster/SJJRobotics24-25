package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TestDrive extends LinearOpMode implements DriveMainAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "init");

        loadMotors(hardwareMap, new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeftDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.getMotor().setDirection(DcMotorSimple.Direction.FORWARD);
        backLeftDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);
        backRightDrive.getMotor().setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
        waitForStart();

        telemetry.addData("drive", "frontLeftDrive");
        telemetry.update();
        frontLeftDrive.setPower(0.4);
        waitMe(3);
        frontLeftDrive.setPower(0);


        telemetry.addData("drive", "frontRightDrive");
        telemetry.update();
        frontRightDrive.setPower(0.4);
        waitMe(3);
        frontRightDrive.setPower(0);

        telemetry.addData("drive", "backLeftDrive");
        telemetry.update();
        backLeftDrive.setPower(0.4);
        waitMe(3);
        backLeftDrive.setPower(0);

        telemetry.addData("drive", "backRightDrive");
        telemetry.update();
        backRightDrive.setPower(0.4);
        waitMe(3);
        backRightDrive.setPower(0);

        waitMe(3);
        frontRightDrive.setPower(0.4);
        frontLeftDrive.setPower(0.4);
        backLeftDrive.setPower(0.4);
        backRightDrive.setPower(0.4);
        waitMe(3);
        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }
    private void waitMe(double sec){
        runtime.reset();
        while (runtime.seconds() < sec) {
        }
    }
}