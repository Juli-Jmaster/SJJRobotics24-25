package org.firstinspires.ftc.teamcode.auto.caden;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.libraries.MovementCurves.MovementCurves;

import static java.lang.Math.*;


public class AutoRobot {
    private DcMotor backRightDrive;
    private DcMotor frontRightDrive;
    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;

    private Servo outtakeAngle;
    private final double OUTTAKE_ANGLE_DROP_POSITION = .589+.03;
    private final double OUTTAKE_ANGLE_LOAD_POSITION = .441+0.028;
    private final double OUTTAKE_ANGLE_READY_LOAD_POSITION = .443+0.028;

    private Servo outtakeClaw;
    private final double OUTTAKE_CLAW_OPEN_POSITION = 0.2;
    private final double OUTTAKE_CLAW_CLOSED_POSITION = 0.34;


    private Servo intakeAngle;
    private final double INTAKE_ANGLE_LOAD_POSITION = .75;
    private final double INTAKE_ANGLE_GRAB_POSITION = .06;

    private Servo intakeClaw;
    private final double INTAKE_CLAW_OPEN_POSITION = .0;
    private final double INTAKE_CLAW_CLOSED_POSITION = 0.052;

    //private Servo intakeSlide1; needs to be implemented
    //private Servo intakeSlide2; needs to be implemented

    private DcMotor elevator1;
    private DcMotor elevator2;

    private DcMotor forwardOdometry;

    private IMU imu;


    private static final double WHEEL_DIAMETER = 48; // In milimeters
    private static final double TICKS_PER_REVOLUTION = 1120;

    //private static final int TICKS_PER_INCH = 45;

    private static final int TICKS_PER_INCH = 337;



    Telemetry telemetry;
    //private double current robotX; unable to reliably solve
    //private double current robotY;


    //this function will move the robot x distance and y distance, and make it face the direction
    //currently does not work correctly
    public void driveRelative(double direction, double x, double y) {


        long time = (long) (1000000000 * sqrt(x * x + y * y));
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;
        double timeAlotted;


        double rX = 1;
        final double MULTIPLIER = .01; //adjust this value to make the robot move less or more,
        // smaller values make it move farther
        double moveSpeed = 0;
        double currentYaw;
        double angleDifference = 180;

        if (abs(x) > abs(y) && x != 0) {
            x = signum(x);
            y = y / abs(x);
        }
        if (abs(y) > abs(x) && y != 0) {
            x = x / abs(y);
            y = signum(y);
        }

        //     Vector2D toGo = new Vector2D(x, y);
        if (direction > 180) {
            direction -= 360;
        }


        do {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            currentTime = System.nanoTime();


            angleDifference = currentYaw - direction;

            if (angleDifference > 1 || angleDifference < -1) {


                angleDifference = currentYaw - direction;

                if (angleDifference > 180) {
                    angleDifference -= 360;
                }

                if (angleDifference < -180) {
                    angleDifference += 360;
                }

                if (angleDifference > 0) {
                    rX = .3 * MovementCurves.circleCurve(angleDifference / 360);
                }
                ;
                if (angleDifference < 0) {
                    angleDifference *= -1;
                    rX = -.3 * MovementCurves.circleCurve(angleDifference / 360);
                }
                ;


                if (rX > 0 && rX < .1) {
                    rX = .1;
                }
                if (rX < 0 && rX > -.1) {
                    rX = -.1;
                }

            } else {
                rX = 0;
            }


            if (currentTime < totalTime) {
                timeAlotted = (totalTime - currentTime) / ((double) (time));
                //       toGo.setVector(x, y);
                //       toGo.adjustAngle(currentYaw);
                moveSpeed = .3 * sin(PI * (timeAlotted));
                //       toGo.scaleVector(moveSpeed);
            } else {
                //     toGo.scaleVector(0);
            }

            // frontRightDrive.setPower(toGo.getJ() - toGo.getI() - rX); //double check these values
            // frontLeftDrive.setPower(toGo.getJ() + toGo.getI() + rX);
            // backLeftDrive.setPower(toGo.getJ() - toGo.getI() + rX);
            // backRightDrive.setPower(toGo.getJ() + toGo.getI() - rX);


            //      telemetry.addData("toGoI", toGo.getI());
            //      telemetry.addData("toGoJ", toGo.getJ());
            //      telemetry.addData("toGoAngle", toGo.getAngle());
            //      telemetry.addData("target", direction);
            telemetry.addData("current", currentYaw);
            telemetry.addData("power", rX);
            telemetry.addData("moveSpeed", moveSpeed);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.update();


        } while (rX != 0 || currentTime < totalTime);

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    //drives forward for a certain amount of inches

    public void driveForwardsInchesIMU(double inches) {
        driveForwardsInchesIMU(inches, 1);
    }

    public void driveForwardsInchesIMU(double inches, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.EXPEASEOUT;
        driveForwardsInchesIMU(inches, powerMultiplier, DEFAULTMOVEMENTCURVE);
    }

    public void driveForwardsInchesIMU(double inches, double powerMultiplier, int movementCurve) {


        final double ADJUSTVALUE = .01;
        final double LOWTHRESHOLD = .25;
        forwardOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int traveledDistance = 0;
        final double TARGETFACING = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;

        final double TOTALDISTANCE = inches*TICKS_PER_INCH;
        double percentTraveled;

        double power;
        double leftAdjust;
        double rightAdjust;

        while (traveledDistance < TOTALDISTANCE) {

            percentTraveled = ((double)traveledDistance)/TOTALDISTANCE;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(percentTraveled);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(percentTraveled);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(percentTraveled);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(percentTraveled);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(percentTraveled);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(percentTraveled);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(percentTraveled);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(percentTraveled);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(percentTraveled);
                    break;
                default:
                    power = MovementCurves.linear(percentTraveled);

            }

            power *= powerMultiplier;

            if(TARGETFACING - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > 20 || TARGETFACING - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -20) {
                leftAdjust = 0;
                rightAdjust = 0;
            } else if (TARGETFACING < imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) {
                leftAdjust = ADJUSTVALUE;
                rightAdjust = 0;
            } else if (TARGETFACING > imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) {
                rightAdjust = ADJUSTVALUE;
                leftAdjust = 0;
            } else {
                leftAdjust = 0;
                rightAdjust = 0;
            }



            if (TOTALDISTANCE-traveledDistance < 2*TICKS_PER_INCH) {
                power = .1;
                leftAdjust = 0;
                rightAdjust = 0;
            } else if (TOTALDISTANCE-traveledDistance < 6*TICKS_PER_INCH){
                power = .2;
            } else if (power < LOWTHRESHOLD) {
                power = LOWTHRESHOLD;
            }

            //assign power to wheels
            frontRightDrive.setPower(power - rightAdjust);
            frontLeftDrive.setPower(power - leftAdjust);
            backLeftDrive.setPower(power - leftAdjust);
            backRightDrive.setPower(power - rightAdjust);

            traveledDistance = forwardOdometry.getCurrentPosition();
        }

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    public void driveBackwardsInchesIMU(double inches) {
        driveBackwardsInchesIMU(inches, 1);
    }

    public void driveBackwardsInchesIMU(double inches, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.EXPEASEOUT;
        driveBackwardsInchesIMU(inches, powerMultiplier, DEFAULTMOVEMENTCURVE);
    }

    public void driveBackwardsInchesIMU(double inches, double powerMultiplier, int movementCurve) {


        final double ADJUSTVALUE = .01;
        final double LOWTHRESHOLD = .25;
        forwardOdometry.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forwardOdometry.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        int traveledDistance = 0;
        final double TARGETFACING = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)+180;

        final double TOTALDISTANCE = inches*TICKS_PER_INCH;
        double percentTraveled;

        double power;
        double leftAdjust;
        double rightAdjust;

        while (traveledDistance < TOTALDISTANCE) {

            percentTraveled = ((double)traveledDistance)/TOTALDISTANCE;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(percentTraveled);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(percentTraveled);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(percentTraveled);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(percentTraveled);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(percentTraveled);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(percentTraveled);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(percentTraveled);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(percentTraveled);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(percentTraveled);
                    break;
                default:
                    power = MovementCurves.linear(percentTraveled);

            }

            power *= powerMultiplier;

            if(TARGETFACING - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > 20 || TARGETFACING - imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -20) {
                leftAdjust = 0;
                rightAdjust = 0;
            } else if (TARGETFACING > imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) {
                leftAdjust = ADJUSTVALUE;
                rightAdjust = 0;
            } else if (TARGETFACING < imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) {
                rightAdjust = ADJUSTVALUE;
                leftAdjust = 0;
            } else {
                leftAdjust = 0;
                rightAdjust = 0;
            }



            if (TOTALDISTANCE-traveledDistance < 2*TICKS_PER_INCH) {
                power = .1;
                leftAdjust = 0;
                rightAdjust = 0;
            } else if (TOTALDISTANCE-traveledDistance < 6*TICKS_PER_INCH){
                power = .2;
            } else if (power < LOWTHRESHOLD) {
                power = LOWTHRESHOLD;
            }

            //assign power to wheels
            frontRightDrive.setPower(-power + rightAdjust);
            frontLeftDrive.setPower(-power + leftAdjust);
            backLeftDrive.setPower(-power + leftAdjust);
            backRightDrive.setPower(-power + rightAdjust);

            traveledDistance = -forwardOdometry.getCurrentPosition();
        }

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }





    public void driveForwardsInches(double inches) {
        driveForwardsInches(inches, 1);
    }

    public void driveForwardsInches(double inches, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.EXPEASEOUT;
        driveForwardsInches(inches, powerMultiplier, DEFAULTMOVEMENTCURVE);
    }

    public void driveForwardsInches(double inches, double powerMultiplier, int movementCurve) {
        //if the wheels get unaligned this will fix it
        final double ADJUSTVALUE = .01;
        final double LOWTHRESHOLD = .25;
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int traveledDistance = 0;
        final double TOTALDISTANCE = inches*TICKS_PER_INCH;
        double percentTraveled;

        double power;
        double leftAdjust;
        double rightAdjust;

        while (traveledDistance < TOTALDISTANCE) {

            percentTraveled = ((double)traveledDistance)/TOTALDISTANCE;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(percentTraveled);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(percentTraveled);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(percentTraveled);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(percentTraveled);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(percentTraveled);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(percentTraveled);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(percentTraveled);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(percentTraveled);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(percentTraveled);
                    break;
                default:
                    power = MovementCurves.linear(percentTraveled);

            }

            power *= powerMultiplier;

            if (frontRightDrive.getCurrentPosition() < frontLeftDrive.getCurrentPosition()) {
                leftAdjust = ADJUSTVALUE;
                rightAdjust = 0;
            } else if (frontLeftDrive.getCurrentPosition() < frontRightDrive.getCurrentPosition()) {
                rightAdjust = ADJUSTVALUE;
                leftAdjust = 0;
            } else {
                leftAdjust = 0;
                rightAdjust = 0;
            }



            if (TOTALDISTANCE-traveledDistance < 2*TICKS_PER_INCH) {
                power = .1;
                leftAdjust = 0;
                rightAdjust = 0;
            } else if (TOTALDISTANCE-traveledDistance < 6*TICKS_PER_INCH){
                power = .2;
            } else if (power < LOWTHRESHOLD) {
                power = LOWTHRESHOLD;
            }

            //assign power to wheels
            frontRightDrive.setPower(power - rightAdjust);
            frontLeftDrive.setPower(power - leftAdjust);
            backLeftDrive.setPower(power - leftAdjust);
            backRightDrive.setPower(power - rightAdjust);

            traveledDistance = (frontLeftDrive.getCurrentPosition()+frontRightDrive.getCurrentPosition())/2;
        }

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }



    public void driveBackwardsInches(double inches) {
        driveBackwardsInches(inches, 1);
    }

    public void driveBackwardsInches(double inches, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.EXPEASEOUT;
        driveBackwardsInches(inches, powerMultiplier, DEFAULTMOVEMENTCURVE);
    }

    public void driveBackwardsInches(double inches, double powerMultiplier, int movementCurve) {
        //if the wheels get unaligned this will fix it
        final double ADJUSTVALUE = .01;
        final double LOWTHRESHOLD = .25;
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int traveledDistance = 0;
        final double TOTALDISTANCE = -inches*TICKS_PER_INCH;
        double percentTraveled;

        double power;
        double leftAdjust;
        double rightAdjust;

        while (traveledDistance > TOTALDISTANCE) {

            percentTraveled = ((double)traveledDistance)/TOTALDISTANCE;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(percentTraveled);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(percentTraveled);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(percentTraveled);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(percentTraveled);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(percentTraveled);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(percentTraveled);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(percentTraveled);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(percentTraveled);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(percentTraveled);
                    break;
                default:
                    power = MovementCurves.linear(percentTraveled);

            }

            power *= powerMultiplier;

            if (frontRightDrive.getCurrentPosition() > frontLeftDrive.getCurrentPosition()) {
                leftAdjust = ADJUSTVALUE;
                rightAdjust = 0;
            } else if (frontLeftDrive.getCurrentPosition() > frontRightDrive.getCurrentPosition()) {
                rightAdjust = ADJUSTVALUE;
                leftAdjust = 0;
            } else {
                leftAdjust = 0;
                rightAdjust = 0;
            }



            if (TOTALDISTANCE-traveledDistance > 2*TICKS_PER_INCH) {
                power = .1;
                leftAdjust = 0;
                rightAdjust = 0;
            } else if (TOTALDISTANCE-traveledDistance > 6*TICKS_PER_INCH){
                power = .2;
            } else if (power < LOWTHRESHOLD) {
                power = LOWTHRESHOLD;
            }

            //assign power to wheels
            frontRightDrive.setPower(-power + rightAdjust);
            frontLeftDrive.setPower(-power + leftAdjust);
            backLeftDrive.setPower(-power + leftAdjust);
            backRightDrive.setPower(-power + rightAdjust);

            traveledDistance = (frontLeftDrive.getCurrentPosition()+frontRightDrive.getCurrentPosition())/2;
        }

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

    }


    //drives forward for a certain amount of seconds

    public void driveForwardsSeconds(double seconds) {

        driveForwardsSeconds(seconds, 1);
        //adjust values to change default behaviour
    }

    public void driveForwardsSeconds(double seconds, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.QUADRATIC;
        driveForwardsSeconds(seconds, powerMultiplier, DEFAULTMOVEMENTCURVE);
        //adjust values to change default behaviour
    }

    public void driveForwardsSeconds(double seconds, double powerMultiplier, int movementCurve) {
        //catch misuse
        if (powerMultiplier > 1) {
            powerMultiplier = 1;
        }
        if (powerMultiplier <= 0) {
            return;
        }

        //initialize variables, and define time frame
        long time = (long) (1000000000 * seconds);
        long startTime = System.nanoTime();
        long totalTime = startTime + time;
        long currentTime = startTime;
        double timeAlotted;
        double power;

        //executing loop
        do  {
            currentTime = System.nanoTime();
            timeAlotted = ((double)(currentTime-startTime) ) / (double)time;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(timeAlotted);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(timeAlotted);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(timeAlotted);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(timeAlotted);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(timeAlotted);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(timeAlotted);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(timeAlotted);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(timeAlotted);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(timeAlotted);
                    break;
                default:
                    power = MovementCurves.linear(timeAlotted);

            }

            power *= powerMultiplier;


            //assign power to wheels
            frontRightDrive.setPower(power);
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(power);
            telemetry.addData("power", power);
            telemetry.addData("timeAlotted", timeAlotted);
            telemetry.update();

        } while (currentTime < totalTime);
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    //drives backwards for a certain amount of seconds

    public void driveBackwardsSeconds(double seconds) {
        driveBackwardsSeconds(seconds, 1);
        //adjust values to change default behaviour
    }

    public void driveBackwardsSeconds(double seconds, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.QUADRATIC;
        driveBackwardsSeconds(seconds, powerMultiplier, DEFAULTMOVEMENTCURVE);
        //adjust values to change default behaviour
    }

    public void driveBackwardsSeconds(double seconds, double powerMultiplier, int movementCurve) {
        //catch misuse
        if (powerMultiplier > 1) {
            powerMultiplier = 1;
        }
        if (powerMultiplier <= 0) {
            return;
        }

        //initialize variables, and define time frame
        long time = (long) (1000000000 * seconds);
        long startTime = System.nanoTime();
        long totalTime = startTime + time;
        long currentTime = startTime;
        double timeAlotted;
        double power;

        //executing loop
        do  {
            currentTime = System.nanoTime();
            timeAlotted = ((double)(currentTime-startTime) ) / (double)time;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(timeAlotted);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(timeAlotted);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(timeAlotted);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(timeAlotted);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(timeAlotted);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(timeAlotted);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(timeAlotted);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(timeAlotted);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(timeAlotted);
                    break;
                default:
                    power = MovementCurves.linear(timeAlotted);

            }

            power *= powerMultiplier;


            //assign power to wheels
            frontRightDrive.setPower(-power);
            frontLeftDrive.setPower(-power);
            backLeftDrive.setPower(-power);
            backRightDrive.setPower(-power);
            telemetry.addData("power", power);
            telemetry.addData("timeAlotted", timeAlotted);
            telemetry.update();

        } while (currentTime < totalTime);
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }

    public void driveRightSeconds(double seconds) {
        driveRightSeconds(seconds, 1);
        //adjust values to change default behaviour
    }

    public void driveRightSeconds(double seconds, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.QUADRATIC;
        driveRightSeconds(seconds, powerMultiplier, DEFAULTMOVEMENTCURVE);
        //adjust values to change default behaviour
    }

    public void driveRightSeconds(double seconds, double powerMultiplier, int movementCurve) {
        //catch misuse
        if (powerMultiplier > 1) {
            powerMultiplier = 1;
        }
        if (powerMultiplier <= 0) {
            return;
        }

        //initialize variables, and define time frame
        long time = (long) (1000000000 * seconds);
        long startTime = System.nanoTime();
        long totalTime = startTime + time;
        long currentTime = startTime;
        double timeAlotted;
        double power;

        //executing loop
        do  {
            currentTime = System.nanoTime();
            timeAlotted = ((double)(currentTime-startTime) ) / (double)time;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(timeAlotted);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(timeAlotted);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(timeAlotted);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(timeAlotted);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(timeAlotted);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(timeAlotted);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(timeAlotted);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(timeAlotted);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(timeAlotted);
                    break;
                default:
                    power = MovementCurves.linear(timeAlotted);

            }

            power *= powerMultiplier;


            //assign power to wheels
            frontRightDrive.setPower(-power);
            frontLeftDrive.setPower(power);
            backLeftDrive.setPower(-power);
            backRightDrive.setPower(power);
            telemetry.addData("power", power);
            telemetry.addData("timeAlotted", timeAlotted);
            telemetry.update();

        } while (currentTime < totalTime);
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }



    public void driveLeftSeconds(double seconds) {

        driveLeftSeconds(seconds, 1);
        //adjust values to change default behaviour
    }

    public void driveLeftSeconds(double seconds, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.QUADRATIC;
        driveLeftSeconds(seconds, powerMultiplier, DEFAULTMOVEMENTCURVE);
        //adjust values to change default behaviour
    }

    public void driveLeftSeconds(double seconds, double powerMultiplier, int movementCurve) {
        //catch misuse
        if (powerMultiplier > 1) {
            powerMultiplier = 1;
        }
        if (powerMultiplier <= 0) {
            return;
        }

        //initialize variables, and define time frame
        long time = (long) (1000000000 * seconds);
        long startTime = System.nanoTime();
        long totalTime = startTime + time;
        long currentTime = startTime;
        double timeAlotted;
        double power;

        //executing loop
        do  {
            currentTime = System.nanoTime();
            timeAlotted = ((double)(currentTime-startTime) ) / (double)time;

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    power = 1;
                    break;
                case MovementCurves.LINEAR:
                    power = MovementCurves.linear(timeAlotted);
                    break;
                case MovementCurves.SIN:
                    power = MovementCurves.sinCurve(timeAlotted);
                    break;
                case MovementCurves.CIRCLE:
                    power = MovementCurves.circleCurve(timeAlotted);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    power = MovementCurves.quadraticCurve(timeAlotted);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    power = MovementCurves.roundedSquareCurve(timeAlotted);
                    break;
                case MovementCurves.PARAMETRIC:
                    power = MovementCurves.parametricCurve(timeAlotted);
                    break;
                case MovementCurves.NORMAL:
                    power = MovementCurves.normalCurve(timeAlotted);
                    break;
                case MovementCurves.EXPEASEIN:
                    power = MovementCurves.exponentialEaseIn(timeAlotted);
                    break;
                case MovementCurves.EXPEASEOUT:
                    power = MovementCurves.exponentialEaseOut(timeAlotted);
                    break;
                default:
                    power = MovementCurves.linear(timeAlotted);

            }

            power *= powerMultiplier;


            //assign power to wheels
            frontRightDrive.setPower(power);
            frontLeftDrive.setPower(-power);
            backLeftDrive.setPower(power);
            backRightDrive.setPower(-power);
            telemetry.addData("power", power);
            telemetry.addData("timeAlotted", timeAlotted);
            telemetry.update();

        } while (currentTime < totalTime);
        frontRightDrive.setPower(0); //double check these values
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }


    public void driveArcLeftForwards(double degrees) {
        driveArcLeftForwards(degrees, 1);
    }

    public void driveArcLeftForwards(double degrees, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.QUADRATIC;
        driveArcLeftForwards(degrees, powerMultiplier, DEFAULTMOVEMENTCURVE);
    }


    public void driveArcLeftForwards(double degrees, double powerMultiplier, int movementCurve) {


        if (powerMultiplier > 1) {
            powerMultiplier = 1;
        }
        if (powerMultiplier <= 0) {
            return;
        }

        double currentYaw;
        double targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + degrees;
        if (targetYaw > 180) {
            targetYaw -= 360;
        }
        double angleDifference = 180;
        double rX = 1;

        while (angleDifference > .5 || angleDifference < -.5) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            angleDifference = currentYaw - targetYaw;

            if (angleDifference > 180) {
                angleDifference -= 360;
            }

            if (angleDifference < -180) {
                angleDifference += 360;
            }



            if (angleDifference < 0) {
                angleDifference *= -1;

            }

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    rX = 1;
                    break;
                case MovementCurves.LINEAR:
                    rX = MovementCurves.linear(angleDifference/360);
                    break;
                case MovementCurves.SIN:
                    rX = MovementCurves.sinCurve(angleDifference/360);
                    break;
                case MovementCurves.CIRCLE:
                    rX = MovementCurves.circleCurve(angleDifference/360);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    rX = MovementCurves.quadraticCurve(angleDifference/360);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    rX = MovementCurves.roundedSquareCurve(angleDifference/360);
                    break;
                case MovementCurves.PARAMETRIC:
                    rX = MovementCurves.parametricCurve(angleDifference/360);
                    break;
                case MovementCurves.NORMAL:
                    rX = MovementCurves.normalCurve(angleDifference/360);
                    break;
                case MovementCurves.EXPEASEIN:
                    rX = MovementCurves.exponentialEaseIn(angleDifference/360);
                    break;
                case MovementCurves.EXPEASEOUT:
                    rX = MovementCurves.exponentialEaseOut(angleDifference/360);
                    break;
                default:
                    rX = MovementCurves.linear(angleDifference/360);

            }




            if (rX > 0 && rX < .1) {
                rX = .1;
            }
            if (rX < 0 && rX > -.1) {
                rX = -.1;
            }


            telemetry.addData("target", targetYaw);
            telemetry.addData("current", currentYaw);
            telemetry.addData("power", rX);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.update();


            frontRightDrive.setPower(rX*powerMultiplier);
            backRightDrive.setPower(rX*powerMultiplier);

        }

        //handle overshooting from rounded square
        if (movementCurve == MovementCurves.ROUNDEDSQUARE) {
            frontRightDrive.setPower(-.4);
            backRightDrive.setPower(-.4);
            waitSeconds(.015);
        }

        frontRightDrive.setPower(0);
        backRightDrive.setPower(0);

    }

    //drives in an arc around a point left of the robot forwards, input how many degrees around it should go


    public void driveArcRightForwards(double degrees) {
        driveArcRightForwards(degrees, 1);
    }

    public void driveArcRightForwards(double degrees, double powerMultiplier) {
        final int DEFAULTMOVEMENTCURVE = MovementCurves.QUADRATIC;
        driveArcRightForwards(degrees, powerMultiplier, DEFAULTMOVEMENTCURVE);
    }


    public void driveArcRightForwards(double degrees, double powerMultiplier, int movementCurve) {


        if (powerMultiplier > 1) {
            powerMultiplier = 1;
        }
        if (powerMultiplier <= 0) {
            return;
        }

        double currentYaw;
        double targetYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - degrees;
        if (targetYaw < 180) {
            targetYaw += 360;
        }
        double angleDifference = 180;
        double lX = 1;

        while (angleDifference > .5 || angleDifference < -.5) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            angleDifference = currentYaw - targetYaw;

            if (angleDifference > 180) {
                angleDifference -= 360;
            }

            if (angleDifference < -180) {
                angleDifference += 360;
            }



            if (angleDifference < 0) {
                angleDifference *= -1;

            }

            switch (movementCurve) {

                case MovementCurves.CONSTANT:
                    lX = 1;
                    break;
                case MovementCurves.LINEAR:
                    lX = MovementCurves.linear(angleDifference/360);
                    break;
                case MovementCurves.SIN:
                    lX = MovementCurves.sinCurve(angleDifference/360);
                    break;
                case MovementCurves.CIRCLE:
                    lX = MovementCurves.circleCurve(angleDifference/360);
                    break;
                case MovementCurves.QUADRATIC:
                    //feels smooth
                    lX = MovementCurves.quadraticCurve(angleDifference/360);
                    break;
                case MovementCurves.ROUNDEDSQUARE:
                    lX = MovementCurves.roundedSquareCurve(angleDifference/360);
                    break;
                case MovementCurves.PARAMETRIC:
                    lX = MovementCurves.parametricCurve(angleDifference/360);
                    break;
                case MovementCurves.NORMAL:
                    lX = MovementCurves.normalCurve(angleDifference/360);
                    break;
                case MovementCurves.EXPEASEIN:
                    lX = MovementCurves.exponentialEaseIn(angleDifference/360);
                    break;
                case MovementCurves.EXPEASEOUT:
                    lX = MovementCurves.exponentialEaseOut(angleDifference/360);
                    break;
                default:
                    lX = MovementCurves.linear(angleDifference/360);

            }




            if (lX > 0 && lX < .1) {
                lX = .1;
            }
            if (lX < 0 && lX > -.1) {
                lX = -.1;
            }


            telemetry.addData("target", targetYaw);
            telemetry.addData("current", currentYaw);
            telemetry.addData("power", lX);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.update();


            frontLeftDrive.setPower(lX*powerMultiplier);
            backLeftDrive.setPower(lX*powerMultiplier);

        }

        //handle overshooting from rounded square

             /*
               if (movementCurve == MovementCurves.ROUNDEDSQUARE) {
                    frontLeftDrive.setPower(-.4);
                    backLeftDrive.setPower(-.4);
                    waitSeconds(.015);
                }
            */
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);

    }


    //makes the robot face a specific direction relative to the starting position
    //0 is facing initial direction
    public void face(double direction) {

        //forward 0
        //right -90, 270
        //backwards 180, -180
        //left 90

        //may be dependent on hub position

        double currentYaw;
        double angleDifference = 180;
        double rX = 1;

        if (direction > 180) {
            direction -= 360;
        }
        while (angleDifference > .5 || angleDifference < -.5) {
            currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            angleDifference = currentYaw - direction;

            if (angleDifference > 180) {
                angleDifference -= 360;
            }

            if (angleDifference < -180) {
                angleDifference += 360;
            }

            if (angleDifference > 0) {
                rX = MovementCurves.quadraticCurve(angleDifference/360);
            }

            if (angleDifference < 0) {
                rX = -MovementCurves.quadraticCurve(-angleDifference/360);
            }



            if (rX > 0 && rX < .05) {
                rX = .05;
            }
            if (rX < 0 && rX > -.05) {
                rX = -.05;
            }


            telemetry.addData("target", direction);
            telemetry.addData("current", currentYaw);
            telemetry.addData("power", rX);
            telemetry.addData("angleDifference", angleDifference);
            telemetry.update();


            frontRightDrive.setPower(-rX);
            frontLeftDrive.setPower(rX);
            backLeftDrive.setPower(rX);
            backRightDrive.setPower(-rX);

        }

        frontRightDrive.setPower(0);
        frontLeftDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);


    }

    public void waitSeconds(double seconds) {
        long time = (long) (1000000000 * seconds);
        long currentTime = System.nanoTime();
        long totalTime = currentTime + time;

        while(currentTime < totalTime) {
            currentTime = System.nanoTime();
        }


    }

    public void intakeClawOpen() {
        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
    }

    public void intakeClawClose() {
        intakeClaw.setPosition(INTAKE_CLAW_CLOSED_POSITION);
    }

    public void intakeAngleGrab() {
        intakeAngle.setPosition(INTAKE_ANGLE_GRAB_POSITION);
    }

    public void intakeAngleLoad() {
        intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
    }

    public void outtakeClawOpen() {
        outtakeClaw.setPosition(OUTTAKE_CLAW_OPEN_POSITION);
    }

    public void outtakeClawClose() {
        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
    }

    public void outtakeClawReady() {
        outtakeClaw.setPosition(OUTTAKE_ANGLE_READY_LOAD_POSITION);
    }

    public void outtakeAngleLoad() {
        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
    }

    public void outtakeAngleDrop() {
        outtakeAngle.setPosition(OUTTAKE_ANGLE_DROP_POSITION);
    }

    public void grabAndLoadSample() {
        elevatorLoadPosition();
        outtakeAngleLoad();
        intakeClawClose();
        waitSeconds(.2);
        intakeAngleLoad();
        waitSeconds(.5);
        elevatorBottom();
        waitSeconds(.4);
        outtakeClawClose();
        waitSeconds(.2);
        intakeClawOpen();
        waitSeconds(.2);
        elevatorTop();
        outtakeAngleDrop();
        intakeClawOpen();
    }

    public void dropSampleAndReset() {
        outtakeClawOpen();
        waitSeconds(.2);
        outtakeAngleLoad();
        elevatorLoadPosition();
        intakeAngleGrab();
        intakeClawOpen();
    }



    public void elevatorTop() {
        elevator2.setTargetPosition(3300);
        elevator1.setTargetPosition(3300);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void elevatorBottom() {
        elevator2.setTargetPosition(0);
        elevator1.setTargetPosition(0);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void elevatorLoadPosition() {
        elevator2.setTargetPosition(100);
        elevator1.setTargetPosition(100);
        elevator1.setPower(1);
        elevator2.setPower(1);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public IMU getImu() {
        return imu;
    }


    public AutoRobot(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize the hardware devices
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontright");
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backRightDrive = hardwareMap.get(DcMotor.class, "backright");
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontleft");
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftDrive = hardwareMap.get(DcMotor.class, "backleft");
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeAngle = hardwareMap.get(Servo.class, "outtakeAngle");
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeClaw");
        outtakeAngle.setPosition(OUTTAKE_ANGLE_LOAD_POSITION);
        outtakeClaw.setPosition(OUTTAKE_CLAW_CLOSED_POSITION);
        intakeAngle = hardwareMap.get(Servo.class, "intakeAngle");
        intakeAngle.setPosition(INTAKE_ANGLE_LOAD_POSITION);
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw");

        intakeClaw.setPosition(INTAKE_CLAW_OPEN_POSITION);
        //intakeSlide1 = hardwareMap.get(Servo.class, "intakeSlide1");
        //intakeSlide2 = hardwareMap.get(Servo.class, "intakeSlide2");

        elevator1 = hardwareMap.get(DcMotor.class, "elavator1");
        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator2 = hardwareMap.get(DcMotor.class, "elavator2");
        elevator2.setDirection(DcMotorSimple.Direction.FORWARD);
        elevator2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw();


        forwardOdometry = hardwareMap.get(DcMotor.class, "straight");

        this.telemetry = telemetry;

    }

}
