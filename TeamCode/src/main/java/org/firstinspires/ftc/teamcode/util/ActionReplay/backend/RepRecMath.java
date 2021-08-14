package org.firstinspires.ftc.teamcode.util.ActionReplay.backend;

public abstract class RepRecMath {
    public static double interpolateBetween(double lowerNumber, double higherNumber, double percentageLocationBetweenNumbers){ // outputs a number depending on where in between the two numbers via linear interpolation
        return ((higherNumber - lowerNumber) * percentageLocationBetweenNumbers) + lowerNumber; // offsets the difference to where the lower number is 0, then scale the higher number by the percentage between, then add the lower number back in
    }

    public static double getFractionBetween(double firstTime, double secondTime, double currentTime){
        if(secondTime > firstTime) // prevent divide by zero errors by only working if the second time is past the first time
            return (currentTime - firstTime)/(secondTime - firstTime); // shift everything such that the first timestamp is 0, then see what the current time is out of the second timestamp
            // for example: the first timestamp = 2, second = 6, current = 3.  3-2 =1, 6-2 =4, we are currently 1/4 of the waybetween 2 and 6
        else
            return 0;
    }

}