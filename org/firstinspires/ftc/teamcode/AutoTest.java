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
import org.firstinspires.ftc.teamcode.auto.OdometryMotor;
import org.firstinspires.ftc.teamcode.auto.UpdatePowerTypes;

import static org.firstinspires.ftc.teamcode.CheckDriveStraight.isWithinTolerance;
import static org.firstinspires.ftc.teamcode.CheckDriveStraight.turnToCorrectSide;


@Autonomous
public class AutoTest extends LinearOpMode {

    private double rotationLeft = 0;
    private IMU imu;
    private DcMotor frontrightDrive;
    private DcMotor backrightDrive;
    private DcMotor frontleftDrive;
    private DcMotor backleftDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        ElapsedTime runtime = new ElapsedTime();
        //define robots attachments to work
        frontrightDrive = hardwareMap.get(DcMotor.class, "frontright");
        backrightDrive = hardwareMap.get(DcMotor.class, "backright");
        frontleftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        backleftDrive = hardwareMap.get(DcMotor.class, "backleft");

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        OdometryMotor straight = new OdometryMotor("straight", OdometryMotor.WHEELTYPE.MM, 48, OdometryMotor.TYPE.TICKPERREV, 2000 );

        Servo outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        Servo outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");

        Servo intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        Servo intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        Servo intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        Servo intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        DcMotor elavator1 = hardwareMap.get(DcMotor.class, "elavator1");
        DcMotor elavator2 = hardwareMap.get(DcMotor.class, "elavator2");

        //reverse correct motor so power of 1 makes robot go forward
        frontrightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        straight.setMotor(hardwareMap.get(DcMotorEx.class, straight.motorname));
        straight.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        imu = hardwareMap.get(IMU.class,"imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

//        final double start = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        imu.resetYaw();
        //send telemetry data and wait for start
        telemetry.update();
        waitForStart();
        runtime.reset();

        boolean forward = true;
        boolean where = false;
        double pos ;

        int target = 180;
        double ticks = 24 / straight.inchesPerCount;
        int position = straight.getMotor().getCurrentPosition() + (int)ticks - (int)equation((int)ticks);// -((int)((1.282*ticks)+336.6)-(int)ticks);
        straight.getMotor().setTargetPosition(position);
        straight.getMotor().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int CUR = straight.getMotor().getCurrentPosition();
        while(!hadssedThes(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())){
            facing(target);
            frontrightDrive.setPower(-UpdatePowerTypes.startEndUpdatePower(straight.getMotor(), CUR)-getRotationLeft(target));
            frontleftDrive.setPower(-UpdatePowerTypes.startEndUpdatePower(straight.getMotor(), CUR)+getRotationLeft(target));
            backleftDrive.setPower(-UpdatePowerTypes.startEndUpdatePower(straight.getMotor(), CUR)+getRotationLeft(target));
            backrightDrive.setPower(-UpdatePowerTypes.startEndUpdatePower(straight.getMotor(), CUR)-getRotationLeft(target));
        }
        while(facing(target)){
            rotateToTarget(target);
        }
        frontleftDrive.setPower(0.0);
        frontrightDrive.setPower(0.0);
        backleftDrive.setPower(0.0);
        backrightDrive.setPower(0.0);
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

    private void wait(double sec, ElapsedTime runtime){
        runtime.reset();
        while(runtime.seconds() < sec){
        }
    }

    private boolean hadssedThes(int input, int target){
        if(target > 0){
            return input >= target;
        } else {
            return input <= target;
        }
    }

    public boolean facing(int target){
        rotationLeft=0;
        boolean correctDriction=false;
        double power = 0.075;
        int cur = (int)imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;
        if (!isWithinTolerance(cur, target, 3)){
            rotationLeft+=power;
            correctDriction=true;
        }
        if (!isWithinTolerance(cur, target, 20)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 40)){
            rotationLeft+=power;
        }
        if (!isWithinTolerance(cur, target, 80)){
            rotationLeft+=power;
        }
        return correctDriction;
    }

    public double getRotationLeft(int target) {
        return rotationLeft*(turnToCorrectSide(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180, target) ? 1 : -1);
    }
    public void rotateToTarget(int target) {
        double rl = getRotationLeft(target);
        backleftDrive.setPower(rl); //backR
        backrightDrive.setPower(-rl); //frontL
        frontleftDrive.setPower(rl);  //frontR
        frontrightDrive.setPower(-rl);
    }

    public double nl(double v){
        return Math.log(v) / Math.log(Math.E);
    }

    public double equation(double v){
        if(v > 12){
            return equation2(v);
        }
        return 273.6772+231.96*nl(v);
    }
    public double equation2(double v){
        return 5.516*v + 817.6;
    }

}
