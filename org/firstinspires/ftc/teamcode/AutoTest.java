package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.DriveMainAuto;
import org.firstinspires.ftc.teamcode.auto.InterfaceErrorIMU;
import org.firstinspires.ftc.teamcode.auto.OdometryMotor;
import org.firstinspires.ftc.teamcode.auto.UpdatePowerTypes;

import static org.firstinspires.ftc.teamcode.auto.CheckDriveStraight.passedTarget;

@Autonomous
public class AutoTest extends LinearOpMode implements DriveMainAuto {


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        ElapsedTime runtime = new ElapsedTime();

        //load motors things
        loadMotors(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));

        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        boolean forward = true;
        boolean where = false;
        double pos ;

        int target = 180;
        straight.move(6);
        int CUR = straight.getMotor().getCurrentPosition();
        while(!passedTarget(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())){
            imu.notFacing(target);
            frontRightDrive.setPower(-UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR)-imu.getRotationLeftPower(target));
            frontLeftDrive.setPower(-UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR)+imu.getRotationLeftPower(target));
            backLeftDrive.setPower(-UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR)+imu.getRotationLeftPower(target));
            backRightDrive.setPower(-UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR)-imu.getRotationLeftPower(target));
        }
        while(passedTarget(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())){
            imu.notFacing(target);
            frontRightDrive.setPower(0.1-imu.getRotationLeftPower(target));
            frontLeftDrive.setPower(0.1+imu.getRotationLeftPower(target));
            backLeftDrive.setPower(0.1+imu.getRotationLeftPower(target));
            backRightDrive.setPower(0.1-imu.getRotationLeftPower(target));
        }
        while(imu.notFacing(target)){
            rotateToTarget(target);
        }
        stopMotors();
        while(opModeIsActive()){
            telemetry.addData("tar", straight.getMotor().getTargetPosition());
            telemetry.addData("cur", straight.getMotor().getCurrentPosition());
            telemetry.addData("zero", straight.getMotor().getCurrentPosition() - straight.getMotor().getTargetPosition());
            telemetry.update();
        }
        // while(opModeIsActive()){
        //     pos=imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        //     frontleftDrive.setPower(0.2);
        //     backleftDrive.setPower(0.2);
        //     frontrightDrive.setPower(-0.2);
        //     backrightDrive.setPower(-0.2);
        //     if ((pos < 5 && pos > -5) && forward==false){
        //         frontleftDrive.setPower(0);
        //         backleftDrive.setPower(0);
        //         frontrightDrive.setPower(0);
        //         backrightDrive.setPower(0);
        //         wait(5, runtime);
        //         forward=true;
        //         where=true;
        //     }
        //     if ((pos < -175 &&  pos > 175) && forward==true){
        //         frontleftDrive.setPower(0);
        //         backleftDrive.setPower(0);
        //         frontrightDrive.setPower(0);
        //         backrightDrive.setPower(0);
        //         wait(5, runtime);
        //         forward=false;
        //         where=true;
        //     }
        //     telemetry.addData("x", pos);
        //     telemetry.addData("forward", forward);
        //     telemetry.addData("cur",  imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //     telemetry.addData("where", where);
        //     telemetry.update();
        // }
    }

    public void rotateToTarget(int target) {
        double rl = imu.getRotationLeftPower(target);
        backLeftDrive.setPower(rl); //backR
        backRightDrive.setPower(-rl); //frontL
        frontLeftDrive.setPower(rl);  //frontR
        frontRightDrive.setPower(-rl);
    }

    public double naturalLog(double v){
        return Math.log(v) / Math.log(Math.E);
    }

    public double equation(double v){
        if(v > 12){
            return equation2(v);
        }
        return 273.6772+231.96* naturalLog(v);
    }
    public double equation2(double v){
        return 5.516*v + 817.6;
    }

}
