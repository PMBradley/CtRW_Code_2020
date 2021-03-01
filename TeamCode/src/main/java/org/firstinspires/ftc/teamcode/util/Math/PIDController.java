package org.firstinspires.ftc.teamcode.util.Math;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class PIDController {
    private static final Range2d DEFAULT_I_ACTIVE_RANGE = new Range2d(0, Double.MAX_VALUE); // what is the minimum amount of error present for I to become active? (default 0)

    ElapsedTime localRuntime;

    public PIDCoefficients coefficients;
    public Range2d iActiveErrorRange;

    private double lastError = 0.0;
    private double lastRuntime = 0.0;
    private double integral = 0.0;
    private double lastPosition = 0.0;
    private double lastTarget = 0.0;


    public PIDController(PIDCoefficients coefficients, Range2d iActiveErrorRange){
        this.coefficients = coefficients;
        this.iActiveErrorRange = iActiveErrorRange;

        localRuntime = new ElapsedTime();
    }
    public PIDController(PIDCoefficients coefficients){
        this(coefficients, DEFAULT_I_ACTIVE_RANGE);
    }
    public PIDController(double Kp, double Ki, double Kd, Range2d iActiveErrorRange){
        this(new PIDCoefficients(Kp, Ki, Kd), iActiveErrorRange);
    }
    public PIDController(double Kp, double Ki, double Kd){
        this(new PIDCoefficients(Kp, Ki, Kd), DEFAULT_I_ACTIVE_RANGE);
    }


    public double getOutput(double current, double target){ // ACCEPTS RADIANS
        double error = current - target; // the error is the difference between where we want to be and where we are right now
        double timeDifference = localRuntime.milliseconds() - lastRuntime; // timeDifference is the time since the last runtime


        // multiplied by the timeDifference to prevent wild variation in how much it is increase if cycle time increases/decreases for some reason
        if( Math.abs(error) >= iActiveErrorRange.getMin() && Math.abs(error) <= iActiveErrorRange.getMax() ){
            integral += error * timeDifference; // the integral is the sum of all error over time, and is used to push past unexpected resistance (as if the arm stays in a single position away from the set position for too long, it builds up over time and pushes past the resistance)
        }

        double dError = ((error - lastError) / timeDifference); // the rate of change of the current error, this component creates a smooth approach to the set point

        double output = (coefficients.p * error) + (coefficients.i * integral) + (coefficients.d * dError); // multiply each term by its coefficient, then add together to get the final power


        lastError = error; // update the last error to be the current error
        lastRuntime = localRuntime.milliseconds(); // update the last runtime to be the current runtime
        lastPosition = current;
        lastTarget = target; //update the last target head to be the current target heading

        return output;
    }


    public double getLastError(){
        return lastError;
    }
    public double getLastRuntime(){
        return lastRuntime;
    }
    public double getLastTarget(){
        return lastTarget;
    }
    public double getLastPosition(){
        return lastPosition;
    }
    public double getIntegral(){
        return integral;
    }
}
