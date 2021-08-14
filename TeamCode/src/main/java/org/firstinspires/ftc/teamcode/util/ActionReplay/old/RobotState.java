package org.firstinspires.ftc.teamcode.util.ActionReplay.old;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Scanner;


public class RobotState {
    private static String CONTROLLER_INDICATOR = "C";

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
            //GamepadState newGamepad1State = GamepadState.makeFromCSV(parser.next());
            //GamepadState newGamepad2State = GamepadState.makeFromCSV(parser.next());
            GamepadState newGamepad1State = new GamepadState();
            GamepadState newGamepad2State = new GamepadState();
            parser.useDelimiter(GamepadState.DELIMITER);

            if (parser.hasNext())
                newGamepad1State.a = (parser.next().charAt(0) == 't'); // read in all of the gamepad variables
            if (parser.hasNext())
                newGamepad1State.b = (parser.next().charAt(0) == 't'); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
            if (parser.hasNext())
                newGamepad1State.x = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.y = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.dpad_up = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.dpad_down = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.dpad_left = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.dpad_right = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.left_bumper = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.right_bumper = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.left_stick_button = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.right_stick_button = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.circle = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.triangle = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.square = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.cross = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.start = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.back = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.share = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.guide = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad1State.options = (parser.next().charAt(0) == 't');

            if (parser.hasNext())
                try { newGamepad1State.left_stick_x = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad1State.left_stick_y = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad1State.right_stick_x = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad1State.right_stick_y = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad1State.left_trigger = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad1State.right_trigger = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}

            if (parser.hasNext())
                try { newGamepad1State.id = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){} // the gamepad ID number, not used for control


            if (parser.hasNext())
                newGamepad2State.a = (parser.next().charAt(0) == 't'); // read in all of the gamepad variables
            if (parser.hasNext())
                newGamepad2State.b = (parser.next().charAt(0) == 't'); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
            if (parser.hasNext())
                newGamepad2State.x = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.y = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.dpad_up = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.dpad_down = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.dpad_left = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.dpad_right = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.left_bumper = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.right_bumper = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.left_stick_button = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.right_stick_button = (parser.next().charAt(0) == 't');

            if (parser.hasNext())
                newGamepad2State.circle = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.triangle = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.square = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.cross = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.start = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.back = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.share = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.guide = (parser.next().charAt(0) == 't');
            if (parser.hasNext())
                newGamepad2State.options = (parser.next().charAt(0) == 't');

            if (parser.hasNext())
                try { newGamepad2State.left_stick_x = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad2State.left_stick_y = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad2State.right_stick_x = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad2State.right_stick_y = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad2State.left_trigger = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}
            if (parser.hasNext())
                try { newGamepad2State.right_trigger = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){}

            if (parser.hasNext())
                try { newGamepad1State.id = Double.parseDouble(parser.next().trim()); } catch (NumberFormatException e){} // the gamepad ID number, not used for control




            return new RobotState(timestamp, new Pose2d(x, y, heading), newGamepad1State, newGamepad2State);
        }
        else {
            return new RobotState(timestamp, new Pose2d(x, y, heading)); // create a RobotState object with the timestamp and positional information that was read/ }
        }
    }
}






///*


/*
            if (parser.hasNext()) {
                String s = parser.next();
                newGamepad1State.left_stick_x = new Double(s);
            }
            if (parser.hasNext()){
                String s = parser.next();
                newGamepad1State.left_stick_y = new Double(s);
            }
            if(parser.hasNext()) {
                String s = parser.next();
                newGamepad1State.right_stick_x = new Double(s);
            }
            if(parser.hasNext()) {
                String s = parser.next();
                newGamepad1State.right_stick_y = new Double(s);
            }
            if(parser.hasNext()) {
                String s = parser.next();
                newGamepad1State.left_trigger = new Double(s);
            }
            if(parser.hasNext()) {
                String s = parser.next();
                newGamepad1State.right_trigger = new Double(s);
            }

            if(parser.hasNext()){
                String s = parser.next();
                newGamepad1State.id = new Double(s); // the gamepad ID number, not used for control
            }
/*
            if(parser.hasNext())
                newGamepad2State.a = (parser.next().charAt(0) == 'T'); // read in all of the gamepad variables
            if(parser.hasNext())
                newGamepad2State.b = (parser.next().charAt(0) == 'T'); // this method is inside the GamepadState class so it can access private class variables of a class of the same type
            if(parser.hasNext())
                newGamepad2State.x = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.y = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.dpad_up = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.dpad_down = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.dpad_left = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.dpad_right = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.left_bumper = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.right_bumper = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.left_stick_button = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.right_stick_button = (parser.next().charAt(0) == 'T');

            if(parser.hasNext())
                newGamepad2State.circle = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.triangle = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.square = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.cross = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.start = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.back = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.share = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.guide = (parser.next().charAt(0) == 'T');
            if(parser.hasNext())
                newGamepad2State.options = (parser.next().charAt(0) == 'T');

            if(parser.hasNext())
                newGamepad2State.left_stick_x = Double.parseDouble(parser.next());
            if(parser.hasNext())
                newGamepad2State.left_stick_y = Double.parseDouble(parser.next());
            if(parser.hasNext())
                newGamepad2State.right_stick_x = Double.parseDouble(parser.next());
            if(parser.hasNext())
                newGamepad2State.right_stick_y = Double.parseDouble(parser.next());
            if(parser.hasNext())
                newGamepad2State.left_trigger = Double.parseDouble(parser.next());
            if(parser.hasNext())
                newGamepad2State.right_trigger = Double.parseDouble(parser.next());

            if(parser.hasNext())
                newGamepad2State.id = Double.parseDouble(parser.next()); // the gamepad ID number, not used for control*/
