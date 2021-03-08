package org.firstinspires.ftc.teamcode.util.ActionReplay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Scanner;

public class GamepadState {
    private boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button;
    private boolean circle, triangle, square, cross, start, back, share, guide, options;
    private float left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;
    private float id;

    public GamepadState(){} // an empty constructor here
    public GamepadState(Gamepad gamepad){
        this.a = gamepad.a; // save all of the gamepad variables out
        this.b = gamepad.b;
        this.x = gamepad.x;
        this.y = gamepad.y;
        this.dpad_up = gamepad.dpad_up;
        this.dpad_down = gamepad.dpad_down;
        this.dpad_left = gamepad.dpad_left;
        this.dpad_right = gamepad.dpad_right;
        this.left_bumper = gamepad.left_bumper;
        this.right_bumper = gamepad.right_bumper;
        this.left_stick_button = gamepad.left_stick_button;
        this.right_stick_button = gamepad.right_stick_button;

        this.circle = gamepad.circle;
        this.triangle = gamepad.triangle;
        this.square = gamepad.square;
        this.cross = gamepad.cross;
        this.start = gamepad.start;
        this.back = gamepad.back;
        this.share = gamepad.share;
        this.guide = gamepad.guide;
        this.options = gamepad.options;

        this.left_stick_x = gamepad.left_stick_x;
        this.left_stick_y = gamepad.left_stick_y;
        this.right_stick_x = gamepad.right_stick_y;
        this.right_stick_y = gamepad.right_stick_y;
        this.left_trigger = gamepad.left_trigger;
        this.right_trigger = gamepad.right_trigger;

        this.id = gamepad.id; // the gamepad ID number, not used for control
    }



    public String toCSVLine(){
        return "" + a +
                "," + b +
                "," + x +
                "," + y +
                "," + dpad_up +
                "," + dpad_down +
                "," + dpad_left +
                "," + dpad_right +
                "," + left_bumper +
                "," + right_bumper +
                "," + left_stick_button +
                "," + right_stick_button +
                "," + circle +
                "," + triangle +
                "," + square +
                "," + cross +
                "," + start +
                "," + back +
                "," + share +
                "," + options +
                "," + left_stick_x +
                "," + left_stick_y +
                "," + right_stick_x +
                "," + right_stick_y +
                "," + left_trigger +
                "," + right_trigger +
                "," + id;
    }

    public static GamepadState makeFromScanner(Scanner parser){ // a static method that returns a RobotState object with the values parsed from the input line (can't be called on instances of the object, just on the class itself)
        parser.useDelimiter(","); // the items are separated by a comma

        GamepadState newState = new GamepadState();

        newState.a = parser.nextBoolean(); // read in all of the gamepad variables
        newState.b = parser.nextBoolean(); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
        newState.x = parser.nextBoolean();
        newState.y = parser.nextBoolean();
        newState.dpad_up = parser.nextBoolean();
        newState.dpad_down = parser.nextBoolean();
        newState.dpad_left = parser.nextBoolean();
        newState.dpad_right = parser.nextBoolean();
        newState.left_bumper = parser.nextBoolean();
        newState.right_bumper = parser.nextBoolean();
        newState.left_stick_button = parser.nextBoolean();
        newState.right_stick_button = parser.nextBoolean();

        newState.circle = parser.nextBoolean();
        newState.triangle = parser.nextBoolean();
        newState.square = parser.nextBoolean();
        newState.cross = parser.nextBoolean();
        newState.start = parser.nextBoolean();
        newState.back = parser.nextBoolean();
        newState.share = parser.nextBoolean();
        newState.guide = parser.nextBoolean();
        newState.options = parser.nextBoolean();

        newState.left_stick_x = parser.nextFloat();
        newState.left_stick_y = parser.nextFloat();
        newState.right_stick_x = parser.nextFloat();
        newState.right_stick_y = parser.nextFloat();
        newState.left_trigger = parser.nextFloat();
        newState.right_trigger = parser.nextFloat();

        newState.id = parser.nextFloat(); // the gamepad ID number, not used for control

        return newState;
    }


    // the mile long list of getters
    public boolean x(){return x;}
    public boolean y(){return y;}
    public boolean a(){return a;}
    public boolean b(){return b;}
    public boolean dpad_up(){return dpad_up;}
    public boolean dpad_down(){return dpad_down;}
    public boolean dpad_left(){return dpad_left;}
    public boolean dpad_right(){return dpad_right;}
    public boolean right_bumper(){return right_bumper;}
    public boolean left_bumper(){return left_bumper;}
    public boolean left_stick_button(){return left_stick_button;}
    public boolean right_stick_button(){return right_stick_button;}
    public boolean circle(){return circle;}
    public boolean triangle(){return triangle;}
    public boolean square(){return square;}
    public boolean cross(){return cross;}
    public boolean start(){return start;}
    public boolean back(){return back;}
    public boolean share(){return share;}
    public boolean guide(){return guide;}
    public boolean options(){return options;}
    public float left_stick_x(){return left_stick_x;}
    public float left_stick_y(){return left_stick_y;}
    public float right_stick_x(){return right_stick_x;}
    public float right_stick_y(){return right_stick_y;}
    public float left_trigger(){return left_trigger;}
    public float right_trigger(){return right_trigger;}

    public float getID(){return id;}

}
