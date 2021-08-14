package org.firstinspires.ftc.teamcode.util.ActionReplay.backend;

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
        gamepad1State = new GamepadState();
        gamepad2State = new GamepadState();
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
    public GamepadState getGamepad1State(){return gamepad1State;}
    public GamepadState getGamepad2State(){return gamepad2State;}
    public boolean hasGamepadStates(){return hasGamepadStates;}
    
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
    @Override
    public String toString(){
        return toCSVLine();
    }

    public static RobotState parseFromCSVLine(String CSVLine) { // a static method that returns a RobotState object with the values parsed from the input line (can't be called on instances of the object, just on the class itself)
        Scanner parser = new Scanner(CSVLine); // setup a scanner to parse out the items from this line of text
        parser.useDelimiter(","); // the items are separated by a comma

        double timestamp = parser.nextDouble(); // get each individual item (in order that they are in the file)
        double x = parser.nextDouble();
        double y = parser.nextDouble();
        double heading = parser.nextDouble();


        if ( parser.next().equals(CONTROLLER_INDICATOR) ) { // if still more in this line, there must be gamepad data here
            //GamepadState newGamepad1State = GamepadState.makeFromScanner(parser);
            //GamepadState newGamepad2State = GamepadState.makeFromScanner(parser);
            //GamepadState newGamepad1State = GamepadState.makeFromString(parser.next());
            //GamepadState newGamepad2State = GamepadState.makeFromString(parser.next());
            GamepadState newGamepad1State = new GamepadState();
            GamepadState newGamepad2State = new GamepadState();

            return new RobotState(timestamp, new Pose2d(x, y, heading), newGamepad1State, newGamepad2State);
        }
        else {
            return new RobotState(timestamp, new Pose2d(x, y, heading)); // create a RobotState object with the timestamp and positional information that was read/ }
        }
    }


    public static RobotState getStateBetween(RobotState firstState, RobotState secondState, double effectiveDriveTime, GamepadState gamepad1Override, GamepadState gamepad2Override){
        if( effectiveDriveTime < firstState.getTimestamp() || firstState.getTimestamp() == secondState.getTimestamp()){ // if we are before the timestamp of the first state, just return that state
            return firstState;
        }
        else if( effectiveDriveTime > secondState.getTimestamp() ){ // if we are after the timestamp of the second state, just return that state
            return secondState;
        }


        double firstTimestamp = firstState.getTimestamp();
        double secondTimestamp = secondState.getTimestamp();

        double fractionBetween = RepRecMath.getFractionBetween(firstTimestamp, secondTimestamp, effectiveDriveTime); // shift everything such that the first timestamp is 0, then see what the current time is out of the second timestamp
        // for example: the first timestamp = 2, second = 6, current = 3.  3-2 =1, 6-2 =4, we are currently 1/4 of the waybetween 2 and 6

        Pose2d firstPose = firstState.getPosition();
        Pose2d secondPose = secondState.getPosition();

        double x = RepRecMath.interpolateBetween(firstPose.getX(), secondPose.getX(), fractionBetween); // shift the "line" to the origin, as though firstPose were the base, then multiply by the fraction between, then shift back
        double y = RepRecMath.interpolateBetween(firstPose.getY(), secondPose.getY(), fractionBetween);
        double heading = RepRecMath.interpolateBetween(firstPose.getHeading(), secondPose.getHeading(), fractionBetween);

        if(gamepad1Override != null && gamepad2Override != null)
            return new RobotState(effectiveDriveTime, new Pose2d(x, y, heading), gamepad1Override, gamepad2Override);
        else
            return new RobotState(effectiveDriveTime, new Pose2d(x, y, heading));
    }

    private RobotState getStateBetween(RobotState firstState, RobotState secondState, double effectiveDriveTime) {
        double fractionBetween = RepRecMath.getFractionBetween(firstState.getTimestamp(), secondState.getTimestamp(), effectiveDriveTime);

        if (firstState.hasGamepadStates() && secondState.hasGamepadStates()){
            return getStateBetween(firstState,
                    secondState,
                    effectiveDriveTime,
                    GamepadState.getGamepadstateBetween(firstState.getGamepad1State(), secondState.getGamepad1State(), fractionBetween),
                    GamepadState.getGamepadstateBetween(firstState.getGamepad2State(), secondState.getGamepad2State(), fractionBetween)
            );
        }
        else {
            return getStateBetween(firstState, secondState, effectiveDriveTime, null, null);
        }
    }

}
