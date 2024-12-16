package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp
public class Drive extends LinearOpMode {
    private final double DEADZONE = .1;
    private final double speedDivider = 2;
    private final double maxPower = 0.75;


    public void drive(DcMotor backleftDrive, DcMotor backrightDrive, DcMotor frontleftDrive, DcMotor frontrightDrive) {



       /* if (!((gamepad1.left_stick_y >= DEADZONE) || (gamepad1.left_stick_x >= DEADZONE) || (gamepad1.left_stick_y <= -DEADZONE) || (gamepad1.left_stick_x <= -DEADZONE))) {
            backleftDrive.setPower(0); //backR
            backrightDrive.setPower(0); //frontL
            frontleftDrive.setPower(0);  //frontR
            frontrightDrive.setPower(0);
            return;

        }
        */
        backleftDrive.setPower(backLeftPower()); //backR
        backrightDrive.setPower(backRightPower()); //frontL
        frontleftDrive.setPower(frontLeftPower());  //frontR
        frontrightDrive.setPower(frontRightPower());
        telemetry.addData("drive", gamepad1.left_stick_y);

    }
    private double backLeftPower(){
        double power=RangeLimit(gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x);
        telemetry.addData("backLeftPower", power);
        return power;
    }
    private double backRightPower(){
        double power=RangeLimit(gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x);
        telemetry.addData("backRightPower", power);
        return power;
    }
    private double frontLeftPower(){
        double power=RangeLimit(gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x);
        telemetry.addData("frontLeftPower", power);
        return power;
    }
    private double frontRightPower(){
        double power=RangeLimit(gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x);
        telemetry.addData("frontRightPower", power);
        return power;
    }

    private double RangeLimit(double value){
        double denominator = Math.max(Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.right_stick_x), 1);
        return (value / denominator) * maxPower;

    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        ElapsedTime runtime = new ElapsedTime();
        DcMotor frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        DcMotor backrightDrive = hardwareMap.get(DcMotor.class, "backright");
        DcMotor frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        DcMotor backleftDrive = hardwareMap.get(DcMotor.class, "backleft");



        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            drive(backleftDrive, backrightDrive, frontleftDrive, frontrightDrive);
            telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y", gamepad1.left_stick_y);
            telemetry.update();

        }
    }
}
