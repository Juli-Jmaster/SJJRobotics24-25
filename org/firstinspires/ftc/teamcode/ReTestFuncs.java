package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.auto.BasicRobot;
import org.firstinspires.ftc.teamcode.auto.DriveMainAuto;
import org.firstinspires.ftc.teamcode.auto.UpdatePowerTypes;

import static org.firstinspires.ftc.teamcode.auto.Utils.passedTarget;

@Autonomous
public class ReTestFuncs extends LinearOpMode implements DriveMainAuto, BasicRobot {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        //load motors things
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        loadAll(hardwareMap);

        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.update();
        waitForStart();
        int straightFacing = 180;
        double inches = 20;

        telemetry.addData("Status", "Running");
        telemetry.addData("side", "straight");
        telemetry.addData("movement","Forward");
        telemetry.update();

        straight.move(inches);
        int CUR = straight.getMotor().getCurrentPosition();
        while(!passedTarget(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())) {
            imu.notFacing(straightFacing);
            moveWithCorrection(UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR), straightFacing);
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("side", "straight");
        telemetry.addData("movement","Backward");
        telemetry.update();
        straight.move(inches);
        CUR = straight.getMotor().getCurrentPosition();
        while(!passedTarget(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())) {
            imu.notFacing(straightFacing);
            moveWithCorrection(-UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR), straightFacing);
        }


        telemetry.addData("Status", "Running");
        telemetry.addData("side", "straight");
        telemetry.addData("movement","Right");
        telemetry.update();
        sideways.move(inches);
        CUR = sideways.getMotor().getCurrentPosition();
        while(!passedTarget(sideways.getMotor().getCurrentPosition(), sideways.getMotor().getTargetPosition())){
            imu.notFacing(straightFacing);
            moveWithCorrectionSideways(UpdatePowerTypes.decreaseAtEnd(sideways.getMotor(), CUR), straightFacing);
        }

        telemetry.addData("Status", "Running");
        telemetry.addData("side", "straight");
        telemetry.addData("movement","Left");
        telemetry.update();
        sideways.move(inches);
        CUR = sideways.getMotor().getCurrentPosition();
        while(!passedTarget(sideways.getMotor().getCurrentPosition(), sideways.getMotor().getTargetPosition())){
            imu.notFacing(straightFacing);
            moveWithCorrectionSideways(-UpdatePowerTypes.decreaseAtEnd(sideways.getMotor(), CUR), straightFacing);
        }
    }
}
