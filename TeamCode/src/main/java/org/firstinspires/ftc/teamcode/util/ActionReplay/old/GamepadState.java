package org.firstinspires.ftc.teamcode.util.ActionReplay.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Scanner;

public class GamepadState {
    public boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button;
    public boolean circle, triangle, square, cross, start, back, share, guide, options;
    public double left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;
    public double id;
    public static final String DELIMITER = ",";

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
        this.right_stick_x = gamepad.right_stick_x;
        this.right_stick_y = gamepad.right_stick_y;
        this.left_trigger = gamepad.left_trigger;
        this.right_trigger = gamepad.right_trigger;

        this.id = gamepad.id; // the gamepad ID number, not used for control
    }



    public String toCSVLine(){
        return "" + a +
                DELIMITER + b +
                DELIMITER+ x +
                DELIMITER + y +
                DELIMITER + dpad_up +
                DELIMITER + dpad_down +
                DELIMITER + dpad_left +
                DELIMITER + dpad_right +
                DELIMITER + left_bumper +
                DELIMITER + right_bumper +
                DELIMITER + left_stick_button +
                DELIMITER + right_stick_button +
                DELIMITER + circle +
                DELIMITER + triangle +
                DELIMITER + square +
                DELIMITER + cross +
                DELIMITER + start +
                DELIMITER + back +
                DELIMITER + share +
                DELIMITER + guide +
                DELIMITER + options +
                DELIMITER + left_stick_x +
                DELIMITER + left_stick_y +
                DELIMITER + right_stick_x +
                DELIMITER + right_stick_y +
                DELIMITER + left_trigger +
                DELIMITER + right_trigger +
                DELIMITER + id;
              //  +DELIMITER;
    }

    //public static GamepadState makeFromScanner(Scanner parser){ // a static method that returns a RobotState object with the values parsed from the input line (can't be called on instances of the object, just on the class itself)
    public static GamepadState makeFromCSV(String input){
        Scanner parser = new Scanner(input);
        parser.useDelimiter(DELIMITER); // the items are separated by a comma

        GamepadState newState = new GamepadState();

        /*newState.a = parser.nextBoolean(); // read in all of the gamepad variables
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

        newState.id = parser.nextFloat(); // the gamepad ID number, not used for control*/

        if(parser.hasNext())
            newState.a = (parser.next().charAt(0) == 't'); // read in all of the gamepad variables
        if(parser.hasNext())
            newState.b = (parser.next().charAt(0) == 't'); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
        if(parser.hasNext())
            newState.x = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.y = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.dpad_up = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.dpad_down = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.dpad_left = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.dpad_right = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.left_bumper = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.right_bumper = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.left_stick_button = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.right_stick_button = (parser.next().charAt(0) == 't');

        if(parser.hasNext())
            newState.circle = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.triangle = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.square = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.cross = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.start = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.back = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.share = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.guide = (parser.next().charAt(0) == 't');
        if(parser.hasNext())
            newState.options = (parser.next().charAt(0) == 't');

        try { newState.left_stick_x = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
        try { newState.left_stick_y = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
        try { newState.right_stick_x = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
        try { newState.right_stick_y = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
        try { newState.left_trigger = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
        try { newState.right_trigger = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}

        try { newState.id = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){} // the gamepad ID number, not used for control

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
    public double left_stick_x(){return left_stick_x;}
    public double left_stick_y(){return left_stick_y;}
    public double right_stick_x(){return right_stick_x;}
    public double right_stick_y(){return right_stick_y;}
    public double left_trigger(){return left_trigger;}
    public double right_trigger(){return right_trigger;}

    public double getID(){return id;}

}
