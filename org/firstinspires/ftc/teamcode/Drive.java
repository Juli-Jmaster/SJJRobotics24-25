package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.BasicRobot;
import org.firstinspires.ftc.teamcode.auto.InterfaceErrorIMU;


@TeleOp
public class Drive extends LinearOpMode implements BasicRobot {

    //motors for drive
    private DcMotor frontrightDrive = null;
    private DcMotor backrightDrive = null;
    private DcMotor frontleftDrive = null;
    private DcMotor backleftDrive = null;

    // variables for movement of robot;
    private InterfaceErrorIMU imu = new InterfaceErrorIMU("imu");

    private boolean claws = false;
    private boolean bpressed = true;
    private boolean apressed = true;
    private boolean ypressed = true;
    private boolean gpdpressed = true;
    private boolean abort = false;

    private final double maxPower = 0.6;

    public void drive(){//DcMotor backleftDrive, DcMotor backrightDrive, DcMotor frontleftDrive, DcMotor frontrightDrive) {
        float y=gamepad1.left_stick_y;
        float x=gamepad1.left_stick_y;
        float rx=gamepad1.right_stick_x;
        //driving robot
        double leftbPower;
        double leftfPower;
        double rightfPower;
        double rightbPower;
        double leftPower=0;
        double rightPower=0;
//        if(gamepad1.left_stick_x <= 0.2  || gamepad1.left_stick_x >= -0.2  ) {
//            double drive = gamepad1.left_stick_x;
//            double turn = Range.clip(gamepad1.left_stick_y + 0.08, -1, 1);
//            if (turn == 0.08) {
//                turn = 0;
//            }
//            drive = drive *-1;
//
//            leftPower = Range.clip(drive - turn, -0.65, 0.65);
//            rightPower = Range.clip(drive + turn, -0.65, 0.65);
//
//            leftbPower = -leftPower;
//            leftfPower = -leftPower;
//            rightfPower = -rightPower;
//            rightbPower = -rightPower;
//        }
//        boolean yes = ((rightPower ==0) && (leftPower==0));
//
//        if((gamepad1.right_stick_x <= 0.2 && yes) || (gamepad1.right_stick_x >= -0.2 && yes)) {
//            double urn = Range.clip(gamepad1.right_stick_x + 0.08, -0.65, 0.65);
//            if (urn == 0.08) {
//                urn = 0;
//            }
//            leftbPower = -urn;
//            leftfPower = urn;
//            rightfPower = urn;
//            rightbPower = -urn;
//        }


        backleftDrive.setPower(RangeLimit(x,y,rx,y+x-rx)); //backR
        backrightDrive.setPower(RangeLimit(x,y,rx,y-x+rx)); //frontL
        frontleftDrive.setPower(RangeLimit(x,y,rx,y-x-rx));  //frontR
        frontrightDrive.setPower(RangeLimit(x,y,rx,y+x+rx));


//        telemetry.addData("drive"));
//        imu.getImu().getRobotAngularVelocity(AngleUnit.DEGREES).
    }

    private double RangeLimit(float x,float y, float rx,double value){
        double denominator = Math.max(Math.abs(y) + Math.abs(x)+ Math.abs(rx), 1);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        telemetry.addData("dem", denominator);
        telemetry.addData("value", value /  denominator);

//        if(Math.abs(y) > 0.2){
//            denominator++;
//        }
//        if(Math.abs(x) > 0.2){
//            denominator++;
//        }
//        if(!(rx==0)){
//            denominator++;
//        }
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
        //load other motors
//        DcMotor elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
//        DcMotor elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        elavator1.setMotor(hardwareMap.get(DcMotor.class, elavator1.motorname));
        elavator1.setupMotor();
        elavator2.setMotor(hardwareMap.get(DcMotor.class, elavator2.motorname));

        //load servos
        outtakeAngle.setServo(hardwareMap.get(Servo.class, outtakeAngle.servoName));
        intakeAngle.setServo(hardwareMap.get(Servo.class, intakeAngle.servoName));
        intakeSlide1.setServo(hardwareMap.get(Servo.class, intakeSlide1.servoName));
        intakeSlide2.setServo(hardwareMap.get(Servo.class, intakeSlide2.servoName));
        outtakeClaw.setServo(hardwareMap.get(Servo.class, outtakeClaw.servoName));
        intakeClaw.setServo(hardwareMap.get(Servo.class, intakeClaw.servoName));
//        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
//        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");


        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //set start position for teleOp
        intakeSlide1.startServo();
        intakeSlide2.startServo();
        intakeAngle.set(TRANSFER);
        outtakeAngle.set(TRANSFER);
        intakeClaw.set(OPEN);//intakeClawPosition.open.position);
        outtakeClaw.set(CLOSE);//outtakeClawPosition.close.position);
        claws=false;



        //imu setup
        imu.setImu(hardwareMap.get(IMU.class,"imu"));
        IMU.Parameters parameters =new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.getImu().initialize(parameters);

        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //is used a constant, so it stay at its current position
            double elavatorPower = 0.1;

            //define the controller positions
            if(gamepad1.dpad_down || gamepad1.dpad_up || gamepad1.dpad_right || gamepad2.dpad_left){
                imu.notFacing(180);
                double rl = imu.getRotationLeftPower(180);
                float y = 0;
                float x = 0;
                if (gamepad1.dpad_up){
                    y= -0.35F;
                }
                if (gamepad1.dpad_down){
                    y= 0.35F;
                }
                if (gamepad1.dpad_left){
                    x = 0.35F;
                    backleftDrive.setPower(x + rl); //backR
                    backrightDrive.setPower(x -rl); //frontL
                    frontleftDrive.setPower(x +rl);  //frontR
                    frontrightDrive.setPower(x -rl);
                }
                if (gamepad1.dpad_right){
                    x= -0.35F;
                }
                backleftDrive.setPower(RangeLimit(x,y,0,y - x + rl)); //backR
                backrightDrive.setPower(RangeLimit(x,y,0,y + x -rl)); //frontL
                frontleftDrive.setPower(RangeLimit(x,y,0,y + x +rl));  //frontR
                frontrightDrive.setPower(RangeLimit(x,y,0,y - x -rl));

            } else {
                drive();
            }
            telemetry.addData("Servo Position 2", (double) intakeSlide2.getPos());
            telemetry.addData("Servo Position 1", (double) intakeSlide1.getPos());
            telemetry.addData("preref", elavator2.getMotor().getPower());
            //pressed
            telemetry.addData("bpressed", bpressed);
            telemetry.addData("apressed", apressed);
            telemetry.addData("ypressed", ypressed);

            if (gamepad2.a && apressed == true) {
                if (!claws) {
                    intakeClaw.set(CLOSE);//intakeClawPosition.close.position);
                    waitMe(0.4, runtime);
                    outtakeClaw.set(OPEN);
//                    outtakeClaw.setPosition(outtakeClawPosition.open.position);
                    claws = true;
                } else if (claws) {
                    outtakeClaw.set(CLOSE);
//                    outtakeClaw.setPosition(outtakeClawPosition.close.position);
                    waitMe(0.4, runtime);
                    intakeClaw.set(OPEN);
//                    intakeClaw.setPosition(intakeClawPosition.open.position);
                    claws = false;
                }
                apressed = false;
            }
            if (gamepad2.b && bpressed == true) {
                if (intakeAngle.get(GRAB) + 0.1 > intakeAngle.getPos() && intakeAngle.get(GRAB) - 0.1 < intakeAngle.getPos()) {
                    intakeAngle.set(TRANSFER);
                } else if (intakeAngle.get(TRANSFER) + 0.1 > intakeAngle.getPos() && intakeAngle.get(TRANSFER) - 0.1 < intakeAngle.getPos()) {
                    intakeAngle.set(GRAB);
                }
                bpressed = false;
            }
            if (gamepad2.y && ypressed == true) {
                if (outtakeAngle.get(PLACE) + 0.05 > outtakeAngle.getPos() && outtakeAngle.get(PLACE) - 0.05 < outtakeAngle.getPos()) {
                    outtakeAngle.set(0);
                } else if (outtakeAngle.get(TRANSFER) + 0.05 > outtakeAngle.getPos() && outtakeAngle.get(TRANSFER) - 0.05 < outtakeAngle.getPos()) {
                    outtakeAngle.set(1);
                }
                ypressed = false;
            }
            if (gamepad2.dpad_down && gpdpressed == true) {
                outtakeClaw.set(OPEN);//outtakeClawPosition.open.position);
                intakeClaw.set(CLOSE);//intakeClawPosition.close.position);
                claws=true;
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeAngle.set(TRANSFER);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeSlide1.set(0.0);
                intakeSlide2.set(1.0);
                waitMe(0.6, runtime);
                if(abort){break;}
                outtakeClaw.set(CLOSE);//outtakeClawPosition.close.position);
                waitMe(0.3, runtime);
                if(abort){break;}
                intakeClaw.set(OPEN);//intakeClawPosition.open.position);
                claws=false;

            }
            if (gamepad2.right_trigger > 0.2) {
//                intakeSlide1.setPosition(0.35/*-0.15*/);
//                intakeSlide2.setPosition(0.65/*+0.15*/);

                intakeSlide1.decrease();
                intakeSlide2.increase();
            }

            if (gamepad2.left_trigger > 0.2) {
                intakeSlide1.increase();
                intakeSlide2.decrease();
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
            elavator1.setPower(elavatorPower);
            telemetry.update();
        }
    }
    private void waitMe(double sec, ElapsedTime runtime){
        runtime.reset();
        while (runtime.seconds() < sec && abort==false) {
        }
    }
}

