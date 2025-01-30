package org.firstinspires.ftc.teamcode.auto;

public interface RobotConstants {

    //define all const for servos if needed

    //define all Motor sand servos

    // all drive motor set to brake for Zero Power Behavior
    boolean driveMotorBrake  = true;
    //if all motor have encoders cables attached and want to use those instead
    boolean drivemotorEncoders = false;

    int defaultDriveMovementCurve = MovementCurves.CIRCLE; //TODO: find correct one
    double defaultDriveMultipier = 1;

    int defaultTurnMovementCurve = MovementCurves.CIRCLE; //TODO: find correct one
    double defaultTurnMultipier = 1;

    OdometryMotor.WHEELTYPE diameterLengthType = OdometryMotor.WHEELTYPE.MM;
    int diameterLength = 48;

    //is for either ticks per revolution or parts per revolution.

    //the Odemetry Pods from goBuilda have ticks per revolution
    // they can be found on their website under fact sheet "Encoder Resolution"
    // so that would be "OdometryMotor.TYPE.TICKPERREV"

    //For motors you can use the same thing just it will be in "PPR in the Output Shaft"
    // so that would be "OdometryMotor.TYPE.PPR"
    OdometryMotor.TYPE ticksPerType = OdometryMotor.TYPE.TICKPERREV;
    int ticksPerTypeNumber = 2000;

    // do not modify;used for when running
    int itorator = 0;
    int straight = 0;
    int sideways = 1;

    //used for while in drive loop
    //only use if statements so drive can still work effectively
    default void whileDrive(int MOVE){

    }
}
