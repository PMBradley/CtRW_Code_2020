package org.firstinspires.ftc.teamcode.util.Math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



public abstract class AngleMath {
    public static double clipAngle(double angle){ // makes a radian input value to be between 0 and 2PI
        return clipAngle(angle, AngleUnit.RADIANS);
    }
    public static double clipAngle(double angle, AngleUnit angleUnit){ // makes a radian input value to be between 0 and 2PI
        double circleValue = 2*Math.PI;
        if( angleUnit.equals(AngleUnit.DEGREES) ) // change modes if not using radians
            circleValue = 360;

        while( angle < 0 ) // for as long as the output is less than 0, add 360 (aka  2PI radians)
            angle += circleValue;

        while( angle >= circleValue ) // and do the opposite to get it below 360
            angle -= circleValue;

        return angle;
    }

    public static double getVectorAngle(Vector2d vector){ // will output a radian angle based on the angle of the vector from 0
        return getVectorAngle(vector, AngleUnit.RADIANS);
    }
    public static double getVectorAngle(Vector2d vector, AngleUnit angleUnit){ // will output a (specified angle unit) angle based on the angle of the vector from 0
        double output = 0;
        double x = vector.getX();
        double y = vector.getY();

        if( Math.abs(x) < 0.00001 ){ // if x is essentially 0, don't risk a div 0 error (plus then angle must be either 90 or -90), remember 90 = PI/2 radians
            output = ( Math.PI/2 ) * ( Math.abs(y)/y ); // multiply 90 degrees by either 1 or -1, if y is positive or negative respectively
        }
        else if( x < 0 ){ // if x is negative, the angle is in quadrant 2 or 3 (and arctan only outputs angles in quadrant 1 or 4)
            output = Math.atan( y / x ) + Math.PI; // so to correct for this, add 180 (aka PI radians)
        }
        else { // if in quadrant 1 or 4, atan will work just fine as normal
            output = Math.atan( y / x );
        }

        while(output < 0)
            output += Math.PI*2; // for as long as the output is less than 0, add 360 (aka  2PI radians)

        if(angleUnit.equals(AngleUnit.RADIANS)) // if they want radians, give them radians
            return output;
        else // otherwise, convert to degrees and output that
            return Math.toDegrees(output);
    }

    public static double getVectorMagnitude(Vector2d vector){ // via pythagorean theorem, find the hypotenuse of the triangle formed by the sides x and y to find the length of the line from (0, 0) to (x, y)
        return Math.sqrt( Math.pow(vector.getX(), 2) + Math.pow(vector.getY(), 2) );
    }


}
