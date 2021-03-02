package org.firstinspires.ftc.teamcode.util.ActionReplay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Scanner;

public class GamepadState {
    boolean a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, left_bumper, right_bumper, left_stick_button, right_stick_button;
    boolean circle, triangle, square, cross, start, back, share, guide, options;
    float left_stick_x, left_stick_y, right_stick_x, right_stick_y, left_trigger, right_trigger;
    float id;


    GamepadState(Gamepad gamepad){
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
    public String toCSVSubline(){
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



        return new GamepadState(new Gamepad()); // TODO
    }
}
