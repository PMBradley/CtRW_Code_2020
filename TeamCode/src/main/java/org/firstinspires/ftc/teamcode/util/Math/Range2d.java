package org.firstinspires.ftc.teamcode.util.Math;

import com.acmerobotics.dashboard.config.Config;


@Config
public class Range2d { // a class that neatly holds a minimum and maximum for a 2d range of values
    double min, max;


    public Range2d(){
        this.min = 0;
        this.max = 0;
    }
    public Range2d(double min, double max){
        this.min = min;
        this.max = max;
    }


    public void setMin(double min){this.min = min;}
    public void setMax(double max){this.max = max;}

    public double getMin() {
        return min;
    }
    public double getMax() {
        return max;
    }
}
