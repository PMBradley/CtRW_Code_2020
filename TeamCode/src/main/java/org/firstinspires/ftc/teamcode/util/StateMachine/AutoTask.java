package org.firstinspires.ftc.teamcode.util.StateMachine;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;

import java.util.ArrayList;


public class AutoTask {
    private String taskName = "";
    private TargetDrivePosition taskLocation; // the location that the robot needs to drive to before starting the task list
    private ArrayList<DriveFollowerTask> tasksAtLocation; // the drive tasks when you reach the location
    private int taskPriority; // the lower the task priority number the higher up on the priority list
    private boolean completed = false;

    public AutoTask( String taskName, int taskPriority, TargetDrivePosition taskLocation ){
        this.taskName = taskName;
        this.taskPriority = taskPriority;
        this.taskLocation = taskLocation;

        tasksAtLocation = new ArrayList<DriveFollowerTask>(); // if no tasks at location provided, setup a list with a single wait task to wait 0 milliseconds
        tasksAtLocation.add( new DriveFollowerTask(0) );
    }
    public AutoTask( String taskName, int taskPriority, TargetDrivePosition taskLocation, ArrayList<DriveFollowerTask> tasksAtLocation ){
        this.taskName = taskName;
        this.taskPriority = taskPriority;
        this.taskLocation = taskLocation;
        this.tasksAtLocation = tasksAtLocation;
    }

    public String getTaskName() {
        return taskName;
    }
    public TargetDrivePosition getTaskLocation(){ return taskLocation; }
    public ArrayList<DriveFollowerTask> getTasksAtLocation(){ return tasksAtLocation; }
    public Trajectory getTrajectoryToTask(Pose2d currentPos, TrajectoryConstraints driveConstraints){ // a method that generates a trajectory based on current position and target drive position
        if( taskLocation.isUsingSpline() ){ // if the target position tells us to use a spline path, generate a spline path
            return new TrajectoryBuilder( currentPos, driveConstraints ) // have the trajectory start at the passed current position and use the passed in drive constraints
                    .splineToSplineHeading( taskLocation.getPose2d(), taskLocation.getSplineHeading() ) // spline to the target position, using the spline heading provided within the target position
                    .build();
        }
        else { // if not using a spline path, line to that position instead
            return new TrajectoryBuilder( currentPos, driveConstraints ) // have the trajectory start at the passed current position and use the passed in drive constraints
                    .lineToSplineHeading( taskLocation.getPose2d() ) // translate (and rotate as needed) to the target position
                    .build();
        }
    }
    public int getPriority(){ return taskPriority; }
    public boolean isCompleted(){ return completed; }

    public AutoTask setCompleted(boolean isCompleted){
        this.completed = isCompleted;
        return this;
    }
    public AutoTask setTaskLocation(TargetDrivePosition taskLocation){
        this.taskLocation = taskLocation;
        return this;
    }
    public AutoTask setLocationTasks(ArrayList<DriveFollowerTask> locationTasks){
        this.tasksAtLocation = locationTasks;
        return this;
    }

}
