package org.firstinspires.ftc.teamcode.util.StateMachine;

import com.acmerobotics.roadrunner.drive.Drive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumConstraints;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryConstraints;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class AutoTaskManager {
    ArrayList<AutoTask> autoTasks;
    int taskIndex = 0;
    double priorityInfluence = 1000; // the number that is put to the power of the task's priority then multiplied by the distance


    public AutoTaskManager(){
        autoTasks = new ArrayList<AutoTask>();
    }
    public AutoTaskManager(double priorityInfluence){
        autoTasks = new ArrayList<AutoTask>();
        this.priorityInfluence = priorityInfluence;
    }
    public AutoTaskManager(ArrayList<AutoTask> autoTasksList){
        this.autoTasks = autoTasksList;
    }
    public AutoTaskManager(ArrayList<AutoTask> autoTasksList, double priorityInfluence){
        this.autoTasks = autoTasksList;
        this.priorityInfluence = priorityInfluence;
    }


    public ArrayList<AutoTask> getTasks(){ return autoTasks; }
    public AutoTask getTaskAt (int index){
        if( index < autoTasks.size() ){ // if the current task wouldn't be out of bounds
            return autoTasks.get(index); // return the current task
        }

        return new AutoTask( "OUT_OF_BOUNDS", 10, new TargetDrivePosition() ); // else, as a default return an error task
    }
    public AutoTask getTaskWithName(String name){ // go through each task until we find one with the corresponding name, then return it
        for(AutoTask task : autoTasks){
            if( task.getTaskName().equals(name) ){
                return task;
            }
        }

        return new AutoTask( "NO_MATCH_FOUND", 10, new TargetDrivePosition() ); // else, as a default return an error task
    }
    public AutoTask getCurrentTask(){
        if( taskIndex < autoTasks.size() ){ // if the current task wouldn't be out of bounds
            return autoTasks.get(taskIndex); // return the current task
        }

        return new AutoTask( "OUT_OF_BOUNDS", 10, new TargetDrivePosition() ); // else, as a default return an error task
    }

    public void updateCurrentTaskToClosest (Pose2d currentPos){
        taskIndex = calcNextTaskIndex(currentPos);
    }
    private int calcNextTaskIndex( Pose2d currentPos ){ // calculate the index of next closest incomplete task
        double shortestDistance = Double.MAX_VALUE; // assume that the shortest applicable distance is somewhere less than the max possible double value
        int shortestIndex = 0;

        for(int i = 0; i < autoTasks.size(); i++){ // TODO: find which task is the closest using priority factored distance
            if(!getTaskAt(i).isCompleted()){
                double distance = priorityFactoredDistance( getTaskAt(i), currentPos );

                if(distance < shortestDistance){
                    shortestDistance = distance;
                    shortestIndex = i;
                }
            }
        }

        return shortestIndex;
    }
    private double priorityFactoredDistance( AutoTask task, Pose2d currentPos ){
        return Math.pow(priorityInfluence, task.getPriority()) * (task.getTaskLocation().getDistanceFrom(currentPos) + 1); // multiply distance by the priority influence to the priority power
    }    // the 1 is added in to prevent any errors with priority being ignored should the distance be 0
    public double getDistanceFromTaskLocation( Pose2d currentPos ){ // returns the distance to the current task's task location
        return getCurrentTask().getTaskLocation().getDistanceFrom(currentPos);
    }

    public ArrayList<DriveFollowerTask> generateCurrentTaskDriveTaskList(Pose2d currentPos, DriveConstraints driveConstraints ){ // calculate a drive task to drive to reach the task start position
        ArrayList<DriveFollowerTask> driveTasks = new ArrayList<DriveFollowerTask>();

        AutoTask currentTask = getCurrentTask();


        if( currentTask.getTaskLocation().isUsingSpline() ){ // if the current task's target location wants us to get there using a spline
            driveTasks.add(new DriveFollowerTask(new TrajectoryBuilder(currentPos, driveConstraints) // create a trajectory starting at our current position that uses the drive constraints as limits
                    .splineToSplineHeading( currentTask.getTaskLocation().getPose2d(), currentTask.getTaskLocation().getSplineHeading() ) // spline to the target position using the spline path heading provided
                    .build()
            ));
        }
        else {
            driveTasks.add(new DriveFollowerTask(new TrajectoryBuilder(currentPos, driveConstraints)
                    .lineToSplineHeading( currentTask.getTaskLocation().getPose2d() ) // line to the target position
                    .build()
            ));
        }

        for(DriveFollowerTask locationTask : currentTask.getTasksAtLocation()){ // go through each task in the current location tasks list and add it to the end
            driveTasks.add(locationTask);
        }

        return driveTasks;
    }

    public void markCurrentTaskComplete(){
        getCurrentTask().setCompleted(true);
    }
    public void markTaskWithNameComplete(String name){
        AutoTask targetTask = getTaskAt(0);
        for(AutoTask task : autoTasks){
            if(task.getTaskName().equals(name)){
                targetTask = task;
            }
        }

        targetTask.setCompleted(true);
    }
    public void markTaskWithNameIncomplete(String name){
        AutoTask targetTask = getTaskAt(0);
        for(AutoTask task : autoTasks){
            if(task.getTaskName().equals(name)){
                targetTask = task;
            }
        }

        targetTask.setCompleted(false);
    }
    public void setTaskWithNameLocation(String name, TargetDrivePosition taskLocation){
        AutoTask targetTask = getTaskAt(0);
        for(AutoTask task : autoTasks){
            if(task.getTaskName().equals(name)){
                targetTask = task;
            }
        }

        targetTask.setTaskLocation(taskLocation);
    }
    public void setTaskWithNameLocationTasks(String name, ArrayList<DriveFollowerTask> atLocationTasks){
        AutoTask targetTask = getTaskAt(0);
        for(AutoTask task : autoTasks){
            if(task.getTaskName().equals(name)){
                targetTask = task;
            }
        }

        targetTask.setLocationTasks(atLocationTasks);
    }
}
