package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class NewAuto extends LinearOpMode implements DriveMainAuto {

    public ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        //send data to console
        telemetry.addData("Status", "Initialized");

        //load motors and imu
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));


        sideways.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        straight.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //send update data
        telemetry.update();
        waitForStart();

        // forward(3);
        while (opModeIsActive()) {
            telemetry.addData("xRotationRate", imu.getImu().getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate);
            telemetry.addData("yRotationRate", imu.getImu().getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate);
            telemetry.addData("zRotationRate", imu.getImu().getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate);

            telemetry.update();
        }
    }
}

