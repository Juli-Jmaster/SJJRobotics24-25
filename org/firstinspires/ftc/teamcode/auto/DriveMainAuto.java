package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;
import static org.firstinspires.ftc.teamcode.auto.CheckDriveStraight.passedTarget;

public interface DriveMainAuto extends MotorUtils {

    OdometryMotor straight = new OdometryMotor("straight", OdometryMotor.WHEELTYPE.MM, 48, OdometryMotor.TYPE.TICKPERREV, 2000 );
    OdometryMotor sideways = new OdometryMotor("sideways", OdometryMotor.WHEELTYPE.MM, 48, OdometryMotor.TYPE.TICKPERREV, 2000 );
    InterfaceErrorIMU imu = new InterfaceErrorIMU("imu");
    double turnMaxSpeed = 0.5;
    ElapsedTime runtime = new ElapsedTime();
//    DcMotorEx sidewys;


    default void loadMotors(HardwareMap hardwareMap, ImuOrientationOnRobot imuOrientationOnRobot){
        //load imu
        imu.setImu(hardwareMap.get(IMU.class,"imu"));
        IMU.Parameters parameters = new IMU.Parameters(imuOrientationOnRobot);
        imu.getImu().initialize(parameters);

        //load drive motors
        frontRightDrive.setMotor(hardwareMap.get(DcMotor.class, frontRightDrive.motorname));
        frontRightDrive.setupMotor();
        backRightDrive.setMotor(hardwareMap.get(DcMotor.class, backRightDrive.motorname));
        backRightDrive.setupMotor();
        frontLeftDrive.setMotor(hardwareMap.get(DcMotor.class, frontLeftDrive.motorname));
        frontLeftDrive.setupMotor();
        backLeftDrive.setMotor(hardwareMap.get(DcMotor.class, backLeftDrive.motorname));
        backLeftDrive.setupMotor();

        //set zero power behavior to brake instead of doing nothing
        frontRightDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.getMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //load odometer Motors
        straight.setMotor(hardwareMap.get(DcMotorEx.class, straight.motorname));
        sideways.setMotor(hardwareMap.get(DcMotorEx.class, "sideways"));


        //reseting for a new run
        imu.resetYaw();
        sideways.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        straight.getMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    default void forward(double inches, int straightHeading){movementStraight(inches, 1, straightHeading);}
    default void backward(double inches, int straightHeading){movementStraight(-inches, -1, straightHeading);}

//    default void backwards(double inches){movementStraight(-inches);}
    //default void left(int inches){sidwaysMovement(inches);}

    default void turnLeft(int degrees, boolean opActive, Telemetry telemetry) throws InterruptedException{
        //set motors to move with just power command
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setup; just for the start of loop

        sleep(500);
        Orientation angles = imu.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double scaledSpeed;
        double oldDegreesLeft=degrees;

        //get target heading and correct
        double targetHeading=angles.firstAngle+degrees;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}

        //its degrees left to target heading
        //this is for turning left
        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);
        while(opActive &&
                degreesLeft>1&&
                oldDegreesLeft-degreesLeft>=0) { //check to see if we overshot target

            //set the speed needed
            scaledSpeed=(Math.abs(targetHeading-angles.firstAngle)/degrees)*turnMaxSpeed;
            if(scaledSpeed>turnMaxSpeed){scaledSpeed=turnMaxSpeed;}
            if(scaledSpeed<0.35){scaledSpeed=0.35;}

            //sets the power to motors
            backLeftDrive.setPower(scaledSpeed*1.3); //extra power to back wheels is = *1.3
            backRightDrive.setPower(-1*scaledSpeed*1.3); // right side need to flip the power to turn = -1*
            frontLeftDrive.setPower(scaledSpeed);
            frontRightDrive.setPower(-1*scaledSpeed);

            //save old and reset for next loop
            angles = imu.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*(360-Math.abs(angles.firstAngle-targetHeading))+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*Math.abs(angles.firstAngle-targetHeading);            telemetry.addData(" start", degreesLeft);
            telemetry.update();
        }
        //stop the motors
        stopMotors();

        //set motors to use encoders again
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250); //small pause at end of turn
    }
    default void turnRight(int degrees, boolean opActive, Telemetry telemetry) throws InterruptedException {
        //set motors to move with just power command
        setModeAllDrive(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setup; just for the start of loop
        sleep(500);
        Orientation angles = imu.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double scaledSpeed;
        double oldDegreesLeft=degrees;

        //get target heading and correct
        double targetHeading=angles.firstAngle-degrees;
        if(targetHeading<-180) {targetHeading+=360;}
        if(targetHeading>180){targetHeading-=360;}

        //its degrees left to target heading
        //this is for turning left
        double degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*Math.abs(angles.firstAngle-targetHeading)+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*(360-Math.abs(angles.firstAngle-targetHeading));
        while(opActive &&
                degreesLeft>1&&
                oldDegreesLeft-degreesLeft>=0) { //check to see if we overshot target

            //set the speed needed
            scaledSpeed=(Math.abs(targetHeading-angles.firstAngle)/degrees)*turnMaxSpeed;
            if(scaledSpeed>turnMaxSpeed){scaledSpeed=turnMaxSpeed;}
            if(scaledSpeed<0.35){scaledSpeed=0.35;}

            //sets the power to motors
            backLeftDrive.setPower(-1*scaledSpeed*1.3); //extra power to back wheels is = *1.3
            backRightDrive.setPower(scaledSpeed*1.3); // right side need to flip the power to turn = -1*
            frontLeftDrive.setPower(-1*scaledSpeed);
            frontRightDrive.setPower(scaledSpeed);

            //save old and reset for next loop
            angles = imu.getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            oldDegreesLeft=degreesLeft;
            degreesLeft = ((int)(Math.signum(angles.firstAngle-targetHeading)+1)/2)*Math.abs(angles.firstAngle-targetHeading)+(int)(Math.signum(targetHeading-angles.firstAngle)+1)/2*(360-Math.abs(angles.firstAngle-targetHeading));
            telemetry.update();
        }
        //stop the motors
        stopMotors();

        //set motors to use encoders again
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250); //small pause at end of turn
    }


    default void movementStraight(double inches, int flip, int straightFacing){
        straight.move(inches);
        int CUR = straight.getMotor().getCurrentPosition();
        while(!passedTarget(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())){
            imu.notFacing(straightFacing);
            moveWithCorrection((flip*-1)*UpdatePowerTypes.decreaseAtEnd(straight.getMotor(), CUR), straightFacing);
        }
        while(passedTarget(straight.getMotor().getCurrentPosition(), straight.getMotor().getTargetPosition())){
            imu.notFacing(straightFacing);
            moveWithCorrection(flip*0.1, straightFacing);
        }
        while(imu.notFacing(straightFacing)){
            moveWithCorrection(0.0, straightFacing);
        }
        stopMotors();
    }

    default void sidwaysMovement(double inches, int flip, int straightFacing){
        sideways.move(inches);
        int CUR = sideways.getMotor().getCurrentPosition();
        while(!passedTarget(sideways.getMotor().getCurrentPosition(), sideways.getMotor().getTargetPosition())){
            imu.notFacing(straightFacing);
            moveWithCorrectionSideways((flip*-1)*UpdatePowerTypes.decreaseAtEnd(sideways.getMotor(), CUR), straightFacing);
        }
        while(passedTarget(sideways.getMotor().getCurrentPosition(), sideways.getMotor().getTargetPosition())){
            imu.notFacing(straightFacing);
            moveWithCorrectionSideways(flip*0.15, straightFacing);
        }
        while(imu.notFacing(straightFacing)){
            moveWithCorrection(0.0, straightFacing);
        }
        moveWithCorrection(0.0, straightFacing);
        stopMotors();
    }

    default void startUsingMotors(){
        cleanupMotors();
        setModeAllDrive(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    default void moveWithCorrection(double power, int target){
        double rl = imu.getRotationLeftPower(target);
        backLeftDrive.setPower(power + rl); //backR
        backRightDrive.setPower(power - rl); //frontL
        frontLeftDrive.setPower(power + rl);  //frontR
        frontRightDrive.setPower(power - rl);
    }
    default void moveWithCorrectionSideways(double power, int target){
        double rl = imu.getRotationLeftPower(target);
        backLeftDrive.setPower(power + rl); //backR
        backRightDrive.setPower(-power - rl); //frontL
        frontLeftDrive.setPower(-power + rl);  //frontR
        frontRightDrive.setPower(power - rl);
    }

    default void turnTo(int degree){
        runtime.reset();
        while(imu.notFacing(degree)){
            moveWithCorrection(0.0, degree);
        }
        stopMotors();
    }



}
