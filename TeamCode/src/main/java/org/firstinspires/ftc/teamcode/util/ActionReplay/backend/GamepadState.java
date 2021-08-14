package org.firstinspires.ftc.teamcode.util.ActionReplay.backend;

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
        this.right_stick_x = gamepad.right_stick_x;
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

    public static GamepadState makeFromScanner(Scanner scanner){ // a static method that returns a RobotState object with the values parsed from the input line (can't be called on instances of the object, just on the class itself)
        scanner.useDelimiter(","); // the items are separated by a comma

        GamepadState newState = new GamepadState();

        newState.a = scanner.nextBoolean(); // read in all of the gamepad variables
        newState.b = scanner.nextBoolean(); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
        newState.x = scanner.nextBoolean();
        newState.y = scanner.nextBoolean();
        newState.dpad_up = scanner.nextBoolean();
        newState.dpad_down = scanner.nextBoolean();
        newState.dpad_left = scanner.nextBoolean();
        newState.dpad_right = scanner.nextBoolean();
        newState.left_bumper = scanner.nextBoolean();
        newState.right_bumper = scanner.nextBoolean();
        newState.left_stick_button = scanner.nextBoolean();
        newState.right_stick_button = scanner.nextBoolean();

        newState.circle = scanner.nextBoolean();
        newState.triangle = scanner.nextBoolean();
        newState.square = scanner.nextBoolean();
        newState.cross = scanner.nextBoolean();
        newState.start = scanner.nextBoolean();
        newState.back = scanner.nextBoolean();
        newState.share = scanner.nextBoolean();
        newState.guide = scanner.nextBoolean();
        newState.options = scanner.nextBoolean();

        newState.left_stick_x = (float)scanner.nextDouble();
        newState.left_stick_y = (float)scanner.nextDouble();
        newState.right_stick_x = (float)scanner.nextDouble();
        newState.right_stick_y = (float)scanner.nextDouble();
        newState.left_trigger = (float)scanner.nextDouble();
        newState.right_trigger = (float)scanner.nextDouble();

        newState.id = (float)scanner.nextDouble(); // the gamepad ID number, not used for control*/
        /*newState.a = Boolean.parseBoolean(scanner.next());; // read in all of the gamepad variables
        newState.b = Boolean.parseBoolean(scanner.next()); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
        newState.x = Boolean.parseBoolean(scanner.next());
        newState.y = Boolean.parseBoolean(scanner.next());
        newState.dpad_up = Boolean.parseBoolean(scanner.next());
        newState.dpad_down = Boolean.parseBoolean(scanner.next());
        newState.dpad_left = Boolean.parseBoolean(scanner.next());
        newState.dpad_right = Boolean.parseBoolean(scanner.next());
        newState.left_bumper = Boolean.parseBoolean(scanner.next());
        newState.right_bumper = Boolean.parseBoolean(scanner.next());
        newState.left_stick_button = Boolean.parseBoolean(scanner.next());
        newState.right_stick_button = Boolean.parseBoolean(scanner.next());

        newState.circle = Boolean.parseBoolean(scanner.next());
        newState.triangle = Boolean.parseBoolean(scanner.next());
        newState.square = Boolean.parseBoolean(scanner.next());
        newState.cross = Boolean.parseBoolean(scanner.next());
        newState.start = Boolean.parseBoolean(scanner.next());
        newState.back = Boolean.parseBoolean(scanner.next());
        newState.share = Boolean.parseBoolean(scanner.next());
        newState.guide =  Boolean.parseBoolean(scanner.next());
        newState.options = Boolean.parseBoolean(scanner.next());

        newState.left_stick_x = Float.parseFloat(scanner.next());
        newState.left_stick_y = Float.parseFloat(scanner.next());
        newState.right_stick_x = Float.parseFloat(scanner.next());
        newState.right_stick_y = Float.parseFloat(scanner.next());
        newState.left_trigger = Float.parseFloat(scanner.next());
        newState.right_trigger = Float.parseFloat(scanner.next());

        newState.id = Float.parseFloat(scanner.next()); // the gamepad ID number, not used for control*/


        return newState;
    }
    public GamepadState(GamepadState oldState, float left_stick_x, float left_stick_y, float right_stick_x, float right_stick_y, float left_trigger, float right_trigger){
        this.a = oldState.a; // save all of the gamepad variables out
        this.b = oldState.b;
        this.x = oldState.x;
        this.y = oldState.y;
        this.dpad_up = oldState.dpad_up;
        this.dpad_down = oldState.dpad_down;
        this.dpad_left = oldState.dpad_left;
        this.dpad_right = oldState.dpad_right;
        this.left_bumper = oldState.left_bumper;
        this.right_bumper = oldState.right_bumper;
        this.left_stick_button = oldState.left_stick_button;
        this.right_stick_button = oldState.right_stick_button;

        this.circle = oldState.circle;
        this.triangle = oldState.triangle;
        this.square = oldState.square;
        this.cross = oldState.cross;
        this.start = oldState.start;
        this.back = oldState.back;
        this.share = oldState.share;
        this.guide = oldState.guide;
        this.options = oldState.options;

        this.left_stick_x = left_stick_x;
        this.left_stick_y = left_stick_y;
        this.right_stick_x = right_stick_x;
        this.right_stick_y = right_stick_y;
        this.left_trigger = left_trigger;
        this.right_trigger = right_trigger;

        this.id = oldState.id; // the gamepad ID number, not used for control
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


    public static GamepadState getGamepadstateBetween(GamepadState gamepad1State, GamepadState gamepad2State, double fractionBetween){
        if(gamepad1State == null || gamepad2State == null){ // don't try to interpolate if n
            return gamepad1State;
        }
        else {
            return new GamepadState( gamepad1State,
                    (float)RepRecMath.interpolateBetween(gamepad1State.left_stick_x, gamepad1State.left_stick_x, fractionBetween),
                    (float)RepRecMath.interpolateBetween(gamepad1State.left_stick_y, gamepad1State.left_stick_y, fractionBetween),
                    (float)RepRecMath.interpolateBetween(gamepad1State.right_stick_x, gamepad1State.right_stick_x, fractionBetween),
                    (float)RepRecMath.interpolateBetween(gamepad1State.right_stick_y, gamepad1State.right_stick_y, fractionBetween),
                    (float)RepRecMath.interpolateBetween(gamepad1State.left_trigger, gamepad1State.left_trigger, fractionBetween),
                    (float)RepRecMath.interpolateBetween(gamepad1State.right_trigger, gamepad1State.right_trigger, fractionBetween)
            );
        }
    }
}
