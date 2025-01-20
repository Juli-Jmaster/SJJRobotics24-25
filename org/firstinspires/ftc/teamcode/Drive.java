package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.auto.BasicRobot;
import org.firstinspires.ftc.teamcode.auto.InterfaceErrorIMU;
import org.firstinspires.ftc.teamcode.auto.CheckDriveStraight;



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
    private boolean bPressed = true;
    private boolean aPressed = true;
    private boolean yPressed = true;
    private boolean dpadDownPressed = true;

    private final double maxPower = 0.8;

    public void drive(){//DcMotor backleftDrive, DcMotor backrightDrive, DcMotor frontleftDrive, DcMotor frontrightDrive) {
        // float y=gamepad1.left_stick_x;
        // float x=gamepad1.left_stick_y;
        // float rx=gamepad1.right_stick_x;

        // backleftDrive.setPower(RangeLimit(x,y,rx,y+x-rx)); //backR
        // backrightDrive.setPower(RangeLimit(x,y,rx,y-x+rx)); //frontL
        // frontleftDrive.setPower(RangeLimit(x,y,rx,y-x-rx));  //frontR
        // frontrightDrive.setPower(RangeLimit(x,y,rx,y+x+rx);


        telemetry.addData("x", gamepad1.left_stick_x);
        telemetry.addData("y", gamepad1.left_stick_y);

    }

    private double RangeLimit(float x,float y, float rx,double value){
        double denominator = Math.max(Math.abs(y) + Math.abs(x)+ Math.abs(rx), 1);
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("rx", rx);
        telemetry.addData("dem", denominator);
        telemetry.addData("value", value /  denominator);

        return denominator;
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

        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        //set start position for teleOp
        intakeSlide1.startServo();
        intakeSlide2.startServo();
        intakeAngle.set(TRANSFER);
        outtakeAngle.set(TRANSFER);
        intakeClaw.set(OPEN);
        outtakeClaw.set(CLOSE);
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
            //is used a constant, so it stay a its current position
            //TODO: change to using encoders and breaks
            double elavatorPower = 0.1;

            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            backleftDrive.setPower((y+x-rx)*maxPower); //backR
            backrightDrive.setPower((y-x+rx)*maxPower); //frontL
            frontleftDrive.setPower((y-x-rx)*maxPower);  //frontR
            frontrightDrive.setPower((y+x+rx)*maxPower);


            telemetry.addData("Servo Position 2", (double) intakeSlide2.getPos());
            telemetry.addData("Servo Position 1", (double) intakeSlide1.getPos());
            telemetry.addData("preref", elavator2.getMotor().getPower());
            //pressed
            telemetry.addData("bPressed", bPressed);
            telemetry.addData("aPressed", aPressed);
            telemetry.addData("yPressed", yPressed);
            telemetry.addData("dpadDownPressed", dpadDownPressed);

            if (gamepad2.a && aPressed) {
                if (!claws) {
                    intakeClaw.set(CLOSE);
                    waitMe(0.2, runtime);
                    outtakeClaw.set(OPEN);
                    claws = true;
                } else if (claws) {
                    outtakeClaw.set(CLOSE);
                    waitMe(0.4, runtime);
                    intakeClaw.set(OPEN);
                    claws = false;
                }
                aPressed = false;
            }
            if (gamepad2.b && bPressed) {
                if (CheckDriveStraight.isWithinTolerance(intakeAngle.getPos(), intakeAngle.get(GRAB), 0.1)) {
                    intakeAngle.set(TRANSFER);
                } else if (CheckDriveStraight.isWithinTolerance(intakeAngle.getPos(), intakeAngle.get(TRANSFER), 0.1)) {
                    intakeAngle.set(GRAB);
                }
                bPressed = false;
            }
            if (gamepad2.y && yPressed) {
                if (CheckDriveStraight.isWithinTolerance(outtakeAngle.getPos(), outtakeAngle.get(PLACE), 0.1)) {
                    outtakeAngle.set(TRANSFER);
                } else if (CheckDriveStraight.isWithinTolerance(outtakeAngle.getPos(),outtakeAngle.get(TRANSFER), 0.1 )){
                    outtakeAngle.set(PLACE);
                }
                yPressed = false;
            }

            if (gamepad2.dpad_down && dpadDownPressed) {
                outtakeClaw.set(OPEN);
                intakeClaw.set(CLOSE);
                claws=true;
                waitMe(0.3, runtime);
                intakeAngle.set(TRANSFER);
                waitMe(0.3, runtime);
                intakeSlide1.set(0.0);
                intakeSlide2.set(1.0);
                waitMe(0.6, runtime);
                outtakeClaw.set(CLOSE);
                waitMe(0.3, runtime);
                intakeClaw.set(OPEN);
                claws=false;
            }

            if (gamepad2.right_trigger > 0.2) {
                intakeSlide1.decrease();
                intakeSlide2.increase();
            }

            if (gamepad2.left_trigger > 0.2) {
                intakeSlide1.increase();
                intakeSlide2.decrease();
            }

            //TODO: test increase these for higher power
            if (gamepad2.left_stick_y < -0.2) {
                elavatorPower = 0.9;
            }
            if (gamepad2.left_stick_y > 0.2) {
                elavatorPower = -0.7;
            }

            //if not pressed set it so it be pressed next time its pressed
            if (!gamepad2.b) {
                bPressed = true;
            }
            if (!gamepad2.y) {
                yPressed = true;
            }
            if (!gamepad2.a) {
                aPressed = true;
            }
            if(!gamepad2.dpad_down){
                dpadDownPressed = true;
            }
            if(gamepad2.x) {
                abort = true;
            }

            //set motor power for elevators
            elavator2.setPower(elavatorPower);
            elavator1.setPower(elavatorPower);
            telemetry.update();
        }
    }
    private void waitMe(double sec, ElapsedTime runtime){
        runtime.reset();
        while (runtime.seconds() < sec) {
        }
    }
}