package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


//its a test drive to see if motors need to be reversed
@TeleOp
public class TestDrive extends LinearOpMode implements DriveMainAuto {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "init");

        loadMotors(hardwareMap, new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        telemetry.update();
        waitForStart();

        telemetry.addData("drive", "frontLeftDrive");
        telemetry.update();
        frontLeftDrive.setPower(1);
        wait(5000);
        frontLeftDrive.setPower(0);


        telemetry.addData("drive", "frontRightDrive");
        telemetry.update();
        frontRightDrive.setPower(1);
        wait(5000);
        frontRightDrive.setPower(0);

        telemetry.addData("drive", "backLeftDrive");
        telemetry.update();
        backLeftDrive.setPower(1);
        wait(5000);
        backLeftDrive.setPower(0);

        telemetry.addData("drive", "backRightDrive");
        telemetry.update();
        backRightDrive.setPower(1);
        wait(5000);
        backRightDrive.setPower(0);
    }
}
