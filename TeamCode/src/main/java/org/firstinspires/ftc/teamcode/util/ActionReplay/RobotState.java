package org.firstinspires.ftc.teamcode.util.ActionReplay;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Scanner;


public class RobotState {
    private double timestamp; // the member data that the class holds
    private Pose2d position;

    public RobotState(){ // constructors for the robot state object
        this(0, new Pose2d());
    }
    public RobotState(double timestamp){
        this(timestamp, new Pose2d());
    }
    public RobotState(double timestamp, Pose2d position){
        this.timestamp = timestamp;
        this.position = position;
    }


    public double getTimestamp(){return timestamp;} // a set of variable getters
    public Pose2d getPosition(){return position;}
    public RobotState getCopy() {return new RobotState(timestamp, position);}


    public String toCSVLine(){
        return "" + timestamp + "," + position.getX() + "," + position.getY() + "," + position.getHeading();
    }
    public static RobotState parseFromCSVLine(String CSVLine){ // a static method that returns a RobotState object with the values parsed from the input line (can't be called on instances of the object, just on the class itself)
        Scanner parser = new Scanner(CSVLine); // setup a scanner to parse out the items from this line of text
        parser.useDelimiter(","); // the items are separated by a comma

        double timestamp = parser.nextDouble(); // get each individual item (in order that they are in the file)
        double x = parser.nextDouble();
        double y = parser.nextDouble();
        double heading = parser.nextDouble();

        return new RobotState( timestamp, new Pose2d(x, y, heading) ); // create a RobotState object with the timestamp and positional information that was read
    }
}
