package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.auto.DriveMainAuto;

@Autonomous
public class TestDecelerations extends LinearOpMode implements DriveMainAuto {
    DcMotorEx motor;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        //load motors things
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.update();
        waitForStart();
        
        straight.move(5);
        int CUR = straight.getMotor().getCurrentPosition();
        while(true){
            decreaseAtEnd(straight.getMotor(), CUR);
        }

    }
    public  double decreaseAtEnd(DcMotorEx motor, int startPostionSave, int decelDistance, int offsetOfEnd){
        //if target is smaller
        //than going backwards
        if (motor.getTargetPosition()<startPostionSave){
            return endingDamp(motor.getCurrentPosition()-motor.getTargetPosition(), decelDistance+offsetOfEnd, offsetOfEnd);
        }
        //if target is bigger
        //than going forward
        if (motor.getTargetPosition()>startPostionSave){
            return endingDamp(motor.getTargetPosition()-motor.getCurrentPosition(),  decelDistance+offsetOfEnd, offsetOfEnd);
        }
        return 0;
    }
    public  double decreaseAtEnd(DcMotorEx motor, int startPostionSave) {
        return decreaseAtEnd(motor, startPostionSave, 300, 80);
    }

    private double endingDamp(double cur, int startPos2, int endpos2) {

        // Check if out of range
        if (cur > startPos2) {
            telemetry.addData("power", "0.65");
            telemetry.update();
            return 0.65;
        }
        if (cur < endpos2) {
            telemetry.addData("power", "0.15");
            telemetry.update();
            return 0.15;
        }

        // Calculate damping
        int accDistance = startPos2 - endpos2;
        double distanceOnAcc = cur - endpos2;
        double normalizedDistanceOnAcc = distanceOnAcc / accDistance;
        double powerRange = 0.65 - 0.15;
        double power = powerRange * normalizedDistanceOnAcc + 0.15;
        telemetry.addData("power", power);
        telemetry.update();
        return power;
    }
}
