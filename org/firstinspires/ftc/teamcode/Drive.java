package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.auto.BasicRobot;


@TeleOp
public class Drive extends LinearOpMode implements BasicRobot {

    private DcMotor frontrightDrive = null;
    private DcMotor backrightDrive = null;
    private DcMotor frontleftDrive = null;
    private DcMotor backleftDrive = null;

    private float y ;
    private float x ;
    private float rx;

    private boolean claws = false;
    private boolean bpressed = true;
    private boolean apressed = true;
    private boolean ypressed = true;
    private boolean gpdpressed = true;
    private boolean abort = false;

    private int TRANSFERINTAKE = 0;
    private int RETRIEV = 1;
    private double MOTION = 0.5;
    //"PLACE" EFFECTS THE FLIP OUT, "TRANSFEROUTTAKE" EFFECTS THE TRANSFER POSITION
    private int PLACEOUTTAKE = 1;
    private int TRANSFEROUTTAKE = 0;


    private final double DEADZONE = .1;
    private final double speedDivider = 2;
    private final double maxPower = 0.6;

    private enum intakeClawPosition{

        open(0.0),
        close(0.110);

        private double position;

        intakeClawPosition(double v) {
            this.position = v;
        }
    }

    private enum outtakeClawPosition{

        open(0.036),
        close(0.28);

        private double position;

        outtakeClawPosition(double v) {
            this.position = v;
        }
    }






    public void drive(){//DcMotor backleftDrive, DcMotor backrightDrive, DcMotor frontleftDrive, DcMotor frontrightDrive) {
        if((rx<0)){
            rx=-0.65F;
        }if((rx>0)){
            rx=0.65F;
        }
        backleftDrive.setPower(RangeLimit(y+(x)-rx)); //backR
        backrightDrive.setPower(RangeLimit(y-(x)+rx)); //frontL
        frontleftDrive.setPower(RangeLimit(y-(x)-rx));  //frontR
        frontrightDrive.setPower(RangeLimit(y+(x)+rx));
        telemetry.addData("drive", RangeLimit(y+x+rx));

    }

    private double RangeLimit(double value){
        double denominator = 0;
        
        if(Math.abs(y) > 0.2){
            denominator++;
        }
        if(Math.abs(x) > 0.2){
            denominator++;
        }
        if(!(rx==0)){
            denominator++;
        }
        telemetry.addData(" dem", denominator);
        return (value /  denominator) * maxPower;

    }



    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        ElapsedTime runtime = new ElapsedTime();
        //define robots attachments to work
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        backrightDrive = hardwareMap.get(DcMotor.class, "backright");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        backleftDrive = hardwareMap.get(DcMotor.class, "backleft");

        outtakeAngle.setServo(hardwareMap.get(Servo.class, "outtakeAngle"));
        intakeAngle.setServo(hardwareMap.get(Servo.class, "intakeAngle"));

        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

//        Servo intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Servo intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        Servo intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        DcMotor elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
        DcMotor elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //set start position for teleOp
        intakeAngle.set(TRANSFERINTAKE);
        outtakeAngle.set(TRANSFEROUTTAKE);
        intakeClaw.setPosition(intakeClawPosition.open.position);
        outtakeClaw.setPosition(outtakeClawPosition.close.position);

        IMU imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //is used a constant, so it stay at its current position
            double elavatorPower = 0.1;

            //define the controller positions
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad2.dpad_left){
                y = 0;
                x = 0;
                rx = 0;
                if (gamepad1.dpad_up){
                    y= -0.35F;
                }
                if (gamepad1.dpad_down){
                    y= 0.35F;
                }
                if (gamepad1.dpad_left){
                    x= -0.35F;
                }
                if (gamepad1.dpad_right){
                    x= 0.35F;
                }
                drive();
            } else {
                drive();
            }
            telemetry.addData("Servo Position in", (double) intakeAngle.getPos());
            telemetry.addData("Servo Position out", (double) outtakeAngle.getServo().getPosition());
            telemetry.addData("preref", gamepad1.dpad_left);
            //pressed
            telemetry.addData("bpressed", bpressed);
            telemetry.addData("apressed", apressed);
            telemetry.addData("ypressed", ypressed);

            if (gamepad2.a && apressed == true) {
                if (!claws) {
                    intakeClaw.setPosition(intakeClawPosition.close.position);
                    waitMe(0.4, runtime);
                    outtakeClaw.setPosition(outtakeClawPosition.open.position);
                    claws = true;
                } else if (claws) {
                    outtakeClaw.setPosition(outtakeClawPosition.close.position);
                    waitMe(0.4, runtime);
                    intakeClaw.setPosition(intakeClawPosition.open.position);
                    claws = false;
                }
                apressed = false;
            }
            if (gamepad2.b && bpressed == true) {
                if (intakeAngle.get(RETRIEV) + 0.1 > intakeAngle.getPos() && intakeAngle.get(RETRIEV) - 0.1 < intakeAngle.getPos()) {
                    intakeAngle.set(TRANSFERINTAKE);
                } else if (intakeAngle.get(TRANSFERINTAKE) + 0.1 > intakeAngle.getPos() && intakeAngle.get(TRANSFERINTAKE) - 0.1 < intakeAngle.getPos()) {
                    intakeAngle.set(RETRIEV);
                }
                bpressed = false;
            }
            if (gamepad2.y && ypressed == true) {
                if (outtakeAngle.get(PLACEOUTTAKE) + 0.05 > outtakeAngle.getPos() && outtakeAngle.get(PLACEOUTTAKE) - 0.05 < outtakeAngle.getPos()) {
                    outtakeAngle.set(0);
                } else if (outtakeAngle.get(TRANSFEROUTTAKE) + 0.05 > outtakeAngle.getPos() && outtakeAngle.get(TRANSFEROUTTAKE) - 0.05 < outtakeAngle.getPos()) {
                    outtakeAngle.set(1);
                }
                ypressed = false;
            }
            if (gamepad2.dpad_down && gpdpressed == true) {
                outtakeClaw.setPosition(outtakeClawPosition.open.position);
                intakeClaw.setPosition(intakeClawPosition.close.position);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeAngle.set(TRANSFERINTAKE);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeSlide1.setPosition(0);
                intakeSlide2.setPosition(1);
                waitMe(0.6, runtime);
                if(abort){break;}
                outtakeClaw.setPosition(outtakeClawPosition.close.position);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeClaw.setPosition(intakeClawPosition.open.position);
                claws=false;

            }
            if (gamepad2.right_trigger > 0.2) {
                intakeSlide1.setPosition(0.35/*-0.15*/);
                intakeSlide2.setPosition(0.65/*+0.15*/);
            }

            if (gamepad2.left_trigger > 0.2) {
                intakeSlide1.setPosition(0);
                intakeSlide2.setPosition(1);
            }

            if (gamepad2.left_stick_y < -0.2) {
                elavatorPower = 0.9;
            }
            if (gamepad2.left_stick_y > 0.2) {
                elavatorPower = -0.7;
            }

            //if not pressed button change
            if (!gamepad2.b) {
                bpressed = true;
            }
            if (!gamepad2.y) {
                ypressed = true;
            }
            if (!gamepad2.a) {
                apressed = true;
            }
            if(!gamepad2.dpad_down){
                gpdpressed = true;
            }
            if(gamepad2.x) {
                abort = true;
            }

            //set motor power
            elavator2.setPower(elavatorPower);
            elavator1.setPower(-elavatorPower);
            telemetry.update();
        }
    }
    private void waitMe(double sec, ElapsedTime runtime){
        runtime.reset();
        while (runtime.seconds() < sec && abort==false) {
        }
    }
}

