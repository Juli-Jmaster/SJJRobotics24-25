package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;

// a class for handling odometry motor and it calculations
// it extends the MotorEx class because a odometry motor is not powered and so it use the motor functions in that class
public class OdometryMotor extends MotorEx {

    // the wheel type is the type of units used ton finsi the diameter of the wheel
    // Ex. if the diamemter of the wheel is 48mm then it is WHEELTYPE.MM
    public enum WHEELTYPE{
        MM,
        INCHES
    }

    //this is the type of data the is found on the GoBuilda website
    //which is called Encoder Resolution on the specs of motor or odemetry pod
    //it either "PPR" or "Countable Events per Revolution"
    // "PPR" is TYPE.PPR and "Countable Events per Revolution" is TYPE.TICKPERREV
    public enum TYPE{
        PPR,
        TICKPERREV
    }

    public static double mmToInches = 0.0393701;

    //this is how many inches there are per 1 ticks; will be a fraction or decimal like 0.002968822
    public double inchesPerTick = 0;
    //this is how many ticks there are per 1 inch; will be greator than 1;
    public double ticksPerInch = 0;

    //all the math for converting ticks to inches for any motor
    public OdometryMotor(String name, WHEELTYPE wheeltype, int wheelDiameter, TYPE type, int tick){
        super(name);
        if(wheeltype==WHEELTYPE.MM && type==TYPE.TICKPERREV){
            inchesPerTick = Math.PI * wheelDiameter*mmToInches / tick;
        }
        if(wheeltype==WHEELTYPE.MM && type==TYPE.PPR){
            inchesPerTick = Math.PI * wheelDiameter*mmToInches / (tick * 4);
        }
        if(wheeltype==WHEELTYPE.INCHES && type==TYPE.TICKPERREV){
            inchesPerTick = Math.PI * wheelDiameter / tick;
        }
        if(wheeltype==WHEELTYPE.INCHES && type==TYPE.PPR){
            inchesPerTick = Math.PI * wheelDiameter / (tick * 4);
        }
        ticksPerInch = 1/ inchesPerTick;
    }


    //pass in the ticks and it returns the amount of inches those ticks are
    private double getInches(int ticks){
        return inchesPerTick * ticks;
    }

    ///MIGHT HAVE FIXED ISSUE SO TRY IsBusy()
    public void move(double inches) {
        double ticks = inches * ticksPerInch;
        super.move((int) ticks);
    }

    //possible deletion
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
