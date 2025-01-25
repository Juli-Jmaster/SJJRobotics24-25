package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

// a utils class to load all drive motors and group motor functions
public interface MotorUtils extends RobotConstants {

    Motor frontRightDrive = new Motor("frontright", false, driveMotorBrake, drivemotorEncoders);
    Motor backRightDrive = new Motor("backright", true, driveMotorBrake, drivemotorEncoders);
    Motor frontLeftDrive = new Motor("frontleft", true, driveMotorBrake, drivemotorEncoders);
    Motor backLeftDrive = new Motor("backleft", true, driveMotorBrake, drivemotorEncoders);

    //utils for using encoder
    default void stopDriveMotors() {
        frontRightDrive.stopMotor();
        backRightDrive.stopMotor();
        frontLeftDrive.stopMotor();
        backLeftDrive.stopMotor();
    }

    //used for stopping motors if or while
    //should never use while
    default void driveStopIf(boolean reached) {
        if (reached && !driveMotorBrake) {
            stopDriveMotors();
        }
    }

    default void driveStopWhile(boolean reached) {
        while (reached && !driveMotorBrake) {
            stopDriveMotors();
        }
    }

    default void powerStraightDriveMotors(double x, double rl) {
        frontRightDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        backRightDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        frontLeftDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
        backLeftDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
    }

    default void powerSidewaysDriveMotors(double x, double rl) {
        frontRightDrive.setPower(-MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        backRightDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) - rl);
        frontLeftDrive.setPower(MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
        backLeftDrive.setPower(-MovementCurves.movementCurves(defaultDriveMovementCurve, x) + rl);
    }

    default void setModeAllDrive(DcMotor.RunMode mode) {
        frontRightDrive.setMode(mode);
        frontLeftDrive.setMode(mode);
        backLeftDrive.setMode(mode);
        backRightDrive.setMode(mode);
    }

    //reset the drive motors encoders
    default void cleanupMotors() {
        setModeAllDrive(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    default void setEncoderDriveTarget(int MOVE, int target) {
        if (MOVE == sideways) {
            frontRightDrive.moveNoPower(-target);
            frontLeftDrive.moveNoPower(target);
            backRightDrive.moveNoPower(target);
            backLeftDrive.moveNoPower(-target);
        } else {
            frontRightDrive.moveNoPower(target);
            frontLeftDrive.moveNoPower(target);
            backLeftDrive.moveNoPower(target);
            backRightDrive.moveNoPower(target);
        }
    }
}
