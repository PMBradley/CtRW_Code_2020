package org.firstinspires.ftc.teamcode.util.ActionReplay;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Scanner;


public class RobotState {
    private static String CONTROLLER_INDICATOR = "CONTROLLER_DATA";

    private double timestamp; // the member data that the class holds
    private Pose2d position;
    private GamepadState gamepad1State;
    private GamepadState gamepad2State;
    private boolean hasGamepadStates;


    public RobotState(){ // constructors for the robot state object
        this(0, new Pose2d());
    }
    public RobotState(double timestamp){
        this(timestamp, new Pose2d());
    }
    public RobotState(double timestamp, Pose2d position){
        this.timestamp = timestamp;
        this.position = position;

        this.hasGamepadStates = false;
    }
    public RobotState(double timestamp, GamepadState gamepad1State, GamepadState gamepad2State){
        this(timestamp, new Pose2d(), gamepad1State, gamepad2State);
    }
    public RobotState(double timestamp, Pose2d position, GamepadState gamepad1State, GamepadState gamepad2State){
        this.timestamp = timestamp;
        this.position = position;
        this.gamepad1State = gamepad1State;
        this.gamepad2State = gamepad2State;

        this.hasGamepadStates = true;
    }



    public double getTimestamp(){return timestamp;} // a set of variable getters
    public Pose2d getPosition(){return position;}
    public RobotState getCopy() {
        if(!hasGamepadStates){
            return new RobotState(timestamp, position);
        }
        else {
            return new RobotState(timestamp, position, gamepad1State, gamepad2State);
        }
    }


    public String toCSVLine(){
        if(hasGamepadStates){
            return "" + timestamp + "," + position.getX() + "," + position.getY() + "," + position.getHeading() + "," + CONTROLLER_INDICATOR + "," + gamepad1State.toCSVLine() + "," + gamepad2State.toCSVLine() + ",";
        }
        else {
            return "" + timestamp + "," + position.getX() + "," + position.getY() + "," + position.getHeading() + ",";
        }
    }
    public static RobotState parseFromCSVLine(String CSVLine) { // a static method that returns a RobotState object with the values parsed from the input line (can't be called on instances of the object, just on the class itself)
        Scanner parser = new Scanner(CSVLine); // setup a scanner to parse out the items from this line of text
        parser.useDelimiter(","); // the items are separated by a comma

        double timestamp = parser.nextDouble(); // get each individual item (in order that they are in the file)
        double x = parser.nextDouble();
        double y = parser.nextDouble();
        double heading = parser.nextDouble();


        if ( parser.next().equals(CONTROLLER_INDICATOR) ) { // if still more in this line, there must be gamepad data here
            GamepadState newGamepad1State = GamepadState.makeFromScanner(parser);
            GamepadState newGamepad2State = GamepadState.makeFromScanner(parser);

            return new RobotState(timestamp, new Pose2d(x, y, heading), newGamepad1State, newGamepad2State);
        }
        else {
            return new RobotState(timestamp, new Pose2d(x, y, heading)); // create a RobotState object with the timestamp and positional information that was read/ }

        }
    }
}
